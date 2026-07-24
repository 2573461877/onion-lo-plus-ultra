#include "ros_main.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <rosbag/bag.h>

namespace fs = boost::filesystem;

namespace {

template <typename T>
T ReadUnalignedField(const std::uint8_t* point_data,
                     std::uint32_t field_offset) {
  T value;
  std::memcpy(&value, point_data + field_offset, sizeof(T));
  return value;
}

}  // namespace

Onion_LO::Onion_LO(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  pnh_.param("Map/pcd_save_en", publish_octomap_, false);
  pnh_.param("Map/Save_path", save_path_, false);
  pnh_.param("Map/save_pcd_en", save_pcd_en_, true);
  pnh_.param("Map/save_map_on_shutdown", save_map_on_shutdown_, true);
  pnh_.param("Map/map_binary_compressed", map_binary_compressed_, true);
  pnh_.param("Map/global_map_voxel_size", global_map_voxel_size_, 0.10);
  pnh_.param("Map/publish_global_map", publish_global_map_, true);
  pnh_.param("Map/global_map_publish_interval",
             global_map_publish_interval_, 20);
  pnh_.param("Map/minimum_map_save_points",
             minimum_map_save_points_, 1000);
  pnh_.param("Map/reject_line_like_map", reject_line_like_map_, true);
  pnh_.param("Map/minimum_secondary_extent_ratio",
             minimum_secondary_extent_ratio_, 0.02);
  pnh_.param("Map/max_mapping_linear_speed",
             max_mapping_linear_speed_, 3.0);
  pnh_.param("Map/max_mapping_angular_speed_deg",
             max_mapping_angular_speed_deg_, 240.0);
  pnh_.param("Map/child_frame", child_frame_, std::string("base_link"));
  pnh_.param("Map/odom_frame", odom_frame_, std::string("odom"));
  pnh_.param("Map/localization_mode", localization_mode_, false);
  pnh_.param("Map/update_loaded_map", config_.update_loaded_map, false);
  pnh_.param("Map/loaded_map_max_points_per_voxel",
             config_.loaded_map_max_points_per_voxel, 150);

  int path_max_size = 5000;
  pnh_.param("Map/path_max_size", path_max_size, 5000);
  path_max_size_ = static_cast<std::size_t>(std::max(1, path_max_size));

  std::string configured_map_path;
  pnh_.param("Map/map_path", configured_map_path,
             std::string("results/onion_map.pcd"));
  map_path_ = ResolveMapPath(configured_map_path);

  pnh_.param("Onion/lidar_topic", lidar_topic_,
             std::string("/livox/lidar"));
  pnh_.param("Onion/lidar_subscriber_queue_size",
             lidar_subscriber_queue_size_, 20);
  pnh_.param("Onion/publisher_queue_size",
             publisher_queue_size_, 5);
  pnh_.param("Onion/lidar_type", config_.type,
             std::string("POINTCLOUD2"));
  pnh_.param("Onion/point_time_field", config_.point_time_field,
             std::string("timestamp"));
  pnh_.param("Onion/point_time_scale", config_.point_time_scale, 1e-9);
  pnh_.param("Onion/point_time_is_offset",
             config_.point_time_is_offset, false);
  pnh_.param("Onion/exp_key_num", config_.exp_key_num, 3000);
  pnh_.param("Onion/Resolution_v", config_.Resolution_v, 1.0);
  pnh_.param("Onion/Resolution_h", config_.Resolution_h, 1.0);

  pnh_.param("Traj/voxel_size", config_.voxel_size, 1.0);
  pnh_.param("Traj/init_interval", config_.init_interval, 200.0);
  pnh_.param("Traj/seg_interval", config_.seg_interval, 20.0);
  pnh_.param("Traj/seg_num", config_.seg_num, 3);
  pnh_.param("Traj/kinematic_constrain", config_.kinematic_constrain, 2.0F);
  pnh_.param("Traj/init_pose_weight", config_.init_pose_weight, 1e9);
  pnh_.param("Traj/converge_thresh", config_.converge_thresh, 0.001);
  pnh_.param("Traj/max_iterations", config_.max_iterations, 20);
  pnh_.param("Traj/raw_point_num", config_.raw_point_num, 30000.0);
  pnh_.param("Traj/max_points_per_voxel",
             config_.max_points_per_voxel, 80);
  pnh_.param("Traj/min_registration_inliers",
             config_.min_registration_inliers, 20);
  pnh_.param("Traj/optimizer_damping",
             config_.optimizer_damping, 1e-6);
  pnh_.param("Traj/max_optimizer_translation_increment",
             config_.max_optimizer_translation_increment, 0.50);
  pnh_.param("Traj/max_optimizer_rotation_increment_deg",
             config_.max_optimizer_rotation_increment_deg, 20.0);
  pnh_.param("Traj/max_optimizer_translation_deviation",
             config_.max_optimizer_translation_deviation, 1.0);
  pnh_.param("Traj/max_optimizer_rotation_deviation_deg",
             config_.max_optimizer_rotation_deviation_deg, 30.0);

  pnh_.param("Diagnostics/enabled", diagnostics_enabled_, true);
  pnh_.param("Diagnostics/stop_on_tracking_failure",
             stop_on_tracking_failure_, true);
  pnh_.param("Diagnostics/context_frames",
             diagnostic_context_frames_, 20);
  std::string configured_diagnostic_directory;
  pnh_.param("Diagnostics/output_directory",
             configured_diagnostic_directory,
             std::string("results/diagnostics"));
  diagnostic_output_directory_ =
      ResolveMapPath(configured_diagnostic_directory);
  std::string configured_metrics_path;
  pnh_.param("Diagnostics/metrics_output_path",
             configured_metrics_path, std::string());
  metrics_output_path_ = ResolveMapPath(configured_metrics_path);

  if (global_map_voxel_size_ <= 0.0) {
    throw std::runtime_error(
        "Map/global_map_voxel_size must be greater than zero");
  }
  if (global_map_publish_interval_ <= 0) {
    throw std::runtime_error(
        "Map/global_map_publish_interval must be greater than zero");
  }
  if (minimum_map_save_points_ <= 0) {
    throw std::runtime_error(
        "Map/minimum_map_save_points must be greater than zero");
  }
  if (minimum_secondary_extent_ratio_ < 0.0 ||
      minimum_secondary_extent_ratio_ >= 1.0) {
    throw std::runtime_error(
        "Map/minimum_secondary_extent_ratio must be in [0, 1)");
  }
  if (max_mapping_linear_speed_ <= 0.0 ||
      max_mapping_angular_speed_deg_ <= 0.0) {
    throw std::runtime_error(
        "Map mapping-speed limits must be greater than zero");
  }
  if (diagnostic_context_frames_ <= 0) {
    throw std::runtime_error(
        "Diagnostics/context_frames must be greater than zero");
  }
  if (lidar_subscriber_queue_size_ <= 0) {
    throw std::runtime_error(
        "Onion/lidar_subscriber_queue_size must be greater than zero");
  }
  if (publisher_queue_size_ <= 0) {
    throw std::runtime_error(
        "Onion/publisher_queue_size must be greater than zero");
  }

  if (config_.type != "POINTCLOUD2") {
    throw std::runtime_error(
        "Onion/lidar_type must be POINTCLOUD2 for livox_ros_driver2");
  }

  std::vector<double> initial_pose;
  if (pnh_.getParam("Map/initial_pose", initial_pose)) {
    if (initial_pose.size() == 6) {
      config_.initial_pose = initial_pose;
    } else {
      ROS_WARN("Map/initial_pose must contain [x,y,z,roll,pitch,yaw] in radians; using identity");
    }
  }

  config_.localization_mode = localization_mode_;
  config_.map_path = map_path_;
  if (localization_mode_ && map_path_.empty()) {
    throw std::runtime_error(
        "Map/localization_mode is true but Map/map_path is empty");
  }

  if (!metrics_output_path_.empty()) {
    const fs::path metrics_path(metrics_output_path_);
    boost::system::error_code directory_error;
    if (!metrics_path.parent_path().empty()) {
      fs::create_directories(metrics_path.parent_path(), directory_error);
    }
    if (directory_error) {
      throw std::runtime_error(
          "failed to create metrics output directory: " +
          directory_error.message());
    }
    metrics_output_.open(metrics_output_path_,
                         std::ios::out | std::ios::trunc);
    if (!metrics_output_.is_open()) {
      throw std::runtime_error(
          "failed to open metrics output path: " +
          metrics_output_path_);
    }
    metrics_output_
        << "reset_id,scan_index,stamp_sec,sensor_elapsed_sec,"
           "wall_elapsed_sec,input_delta_ms,processing_ms,"
           "average_processing_ms,min_registration_inliers,"
           "map_points,map_voxels,x,y,z,qx,qy,qz,qw\n";
  }

  global_map_cache_.Configure(global_map_voxel_size_);
  complete_map_.reset(new PointCloudXYZ());
  octomap_cache_.voxel_size_ = 2.0;
  octomap_cache_.reslt_ = 0.2;

  odom_publisher_ =
      pnh_.advertise<nav_msgs::Odometry>("odometry", publisher_queue_size_);
  pose_publisher_ =
      pnh_.advertise<geometry_msgs::PoseStamped>("pose", publisher_queue_size_);
  frame_publisher_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("frame",
                                               publisher_queue_size_);
  local_map_publisher_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("local_map",
                                               publisher_queue_size_);
  global_map_publisher_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
  octomap_pub_ =
      pnh_.advertise<octomap_msgs::Octomap>("octomap_binary",
                                            publisher_queue_size_);
  traj_publisher_ =
      pnh_.advertise<nav_msgs::Path>("trajectory", publisher_queue_size_);
  path_msg_.header.frame_id = odom_frame_;

  trajLOdometry_.reset(new traj::TrajLOdometry(config_));

  sub_lidar_ = nh_.subscribe(lidar_topic_, lidar_subscriber_queue_size_,
                             &Onion_LO::PointCloudCallback, this,
                             ros::TransportHints().tcpNoDelay());
  save_map_server_ =
      pnh_.advertiseService("save_map", &Onion_LO::SaveMapService, this);

  if (localization_mode_) {
    initial_pose_subscriber_ =
        nh_.subscribe("/initialpose", 1, &Onion_LO::InitialPoseCallback, this);
    if (publish_global_map_) {
      PublishLoadedMap();
    }
  }

  if (child_frame_ != "base_link") {
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = child_frame_;
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    broadcaster.sendTransform(transform);
  }

  ROS_INFO_STREAM("\033[32mOnion-LO++ initialized in "
                  << (localization_mode_ ? "localization" : "mapping")
                  << " mode; PointCloud2 topic: " << lidar_topic_
                  << "; registration voxel cap: "
                  << config_.max_points_per_voxel
                  << "; loaded-map voxel cap: "
                  << config_.loaded_map_max_points_per_voxel
                  << "; LiDAR input queue: "
                  << lidar_subscriber_queue_size_
                  << "; publisher queue: "
                  << publisher_queue_size_
                  << "; persistent map voxel: "
                  << global_map_voxel_size_ << " m"
                  << "\033[0m");
}

Onion_LO::~Onion_LO() {
  if (!localization_mode_ && save_pcd_en_ && save_map_on_shutdown_) {
    std::string message;
    if (SaveGlobalMap(&message)) {
      ROS_INFO_STREAM(message);
    } else {
      ROS_WARN_STREAM("Map was not saved on shutdown: " << message);
    }
  }
}

std::string Onion_LO::ResolveMapPath(
    const std::string& configured_path) const {
  if (configured_path.empty()) return {};
  fs::path path(configured_path);
  if (path.is_absolute()) return path.lexically_normal().string();
  return (fs::path(ROOT_DIR) / path).lexically_normal().string();
}

void Onion_LO::PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  BufferDiagnosticFrame(msg);
  try {
    if (!msg->header.frame_id.empty() &&
        msg->header.frame_id != child_frame_) {
      ROS_WARN_STREAM_ONCE(
          "PointCloud2 frame_id is '" << msg->header.frame_id
          << "' but Map/child_frame is '" << child_frame_
          << "'. Onion-LO++ treats input points as the tracked child frame "
             "without applying a TF extrinsic; make these frames identical "
             "or pre-transform the cloud.");
    }
    LiDAR_odom(msg);
  } catch (const std::exception& exception) {
    map_integrity_fault_ = true;
    map_integrity_fault_reason_ = exception.what();
    if (diagnostics_enabled_) {
      DumpTrackingFailure(*msg, exception.what());
    }
    ROS_FATAL("PointCloud2 conversion/odometry stopped: %s",
              exception.what());
    if (stop_on_tracking_failure_) {
      ros::shutdown();
    }
  }
}

void Onion_LO::BufferDiagnosticFrame(
    const sensor_msgs::PointCloud2::ConstPtr& message) {
  if (!diagnostics_enabled_ || !message) return;

  diagnostic_cloud_buffer_.emplace_back(*message);
  while (diagnostic_cloud_buffer_.size() >
         static_cast<std::size_t>(diagnostic_context_frames_)) {
    diagnostic_cloud_buffer_.pop_front();
  }
}

void Onion_LO::DumpTrackingFailure(
    const sensor_msgs::PointCloud2& message,
    const std::string& reason) {
  if (diagnostic_failure_dumped_) return;
  diagnostic_failure_dumped_ = true;

  const fs::path output_directory(diagnostic_output_directory_);
  boost::system::error_code directory_error;
  fs::create_directories(output_directory, directory_error);
  if (directory_error) {
    ROS_ERROR("Failed to create diagnostic directory '%s': %s",
              diagnostic_output_directory_.c_str(),
              directory_error.message().c_str());
    return;
  }

  const std::uint64_t timestamp_ns =
      message.header.stamp.isZero()
          ? ros::WallTime::now().toNSec()
          : message.header.stamp.toNSec();
  const std::string stem = "tracking_failure_" +
                           std::to_string(timestamp_ns);
  const fs::path bag_path = output_directory / (stem + "_context.bag");
  const fs::path map_path =
      output_directory / (stem + "_registration_map.pcd");
  const fs::path report_path = output_directory / (stem + "_report.txt");

  const sensor_msgs::PointField* diagnostic_x = nullptr;
  const sensor_msgs::PointField* diagnostic_y = nullptr;
  const sensor_msgs::PointField* diagnostic_z = nullptr;
  const sensor_msgs::PointField* diagnostic_time = nullptr;
  for (const auto& field : message.fields) {
    if (field.name == "x") diagnostic_x = &field;
    if (field.name == "y") diagnostic_y = &field;
    if (field.name == "z") diagnostic_z = &field;
    if (field.name == config_.point_time_field) diagnostic_time = &field;
  }
  Eigen::Vector3d raw_minimum =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d raw_maximum =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  double minimum_point_time =
      std::numeric_limits<double>::infinity();
  double maximum_point_time =
      -std::numeric_limits<double>::infinity();
  std::size_t finite_point_count = 0;
  const bool can_inspect_points =
      diagnostic_x && diagnostic_y && diagnostic_z && diagnostic_time &&
      diagnostic_x->datatype == sensor_msgs::PointField::FLOAT32 &&
      diagnostic_y->datatype == sensor_msgs::PointField::FLOAT32 &&
      diagnostic_z->datatype == sensor_msgs::PointField::FLOAT32 &&
      diagnostic_time->datatype == sensor_msgs::PointField::FLOAT64 &&
      diagnostic_x->offset + sizeof(float) <= message.point_step &&
      diagnostic_y->offset + sizeof(float) <= message.point_step &&
      diagnostic_z->offset + sizeof(float) <= message.point_step &&
      diagnostic_time->offset + sizeof(double) <= message.point_step &&
      message.row_step >= message.width * message.point_step &&
      message.data.size() >=
          static_cast<std::size_t>(message.row_step) * message.height;
  if (can_inspect_points) {
    const double header_time = message.header.stamp.toSec();
    for (std::uint32_t row = 0; row < message.height; ++row) {
      const std::uint8_t* row_data =
          message.data.data() +
          static_cast<std::size_t>(row) * message.row_step;
      for (std::uint32_t column = 0; column < message.width; ++column) {
        const std::uint8_t* point_data =
            row_data + static_cast<std::size_t>(column) *
                           message.point_step;
        const float x =
            ReadUnalignedField<float>(point_data, diagnostic_x->offset);
        const float y =
            ReadUnalignedField<float>(point_data, diagnostic_y->offset);
        const float z =
            ReadUnalignedField<float>(point_data, diagnostic_z->offset);
        const double raw_time = ReadUnalignedField<double>(
            point_data, diagnostic_time->offset);
        const double scaled_time =
            config_.point_time_is_offset
                ? header_time + raw_time * config_.point_time_scale
                : raw_time * config_.point_time_scale;
        if (!std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(z) || !std::isfinite(scaled_time)) {
          continue;
        }
        const Eigen::Vector3d position(x, y, z);
        raw_minimum = raw_minimum.cwiseMin(position);
        raw_maximum = raw_maximum.cwiseMax(position);
        minimum_point_time = std::min(minimum_point_time, scaled_time);
        maximum_point_time = std::max(maximum_point_time, scaled_time);
        ++finite_point_count;
      }
    }
  }

  std::string bag_status;
  try {
    rosbag::Bag bag;
    bag.open(bag_path.string(), rosbag::bagmode::Write);
    for (const auto& cloud : diagnostic_cloud_buffer_) {
      const ros::Time write_time =
          cloud.header.stamp.isZero() ? ros::Time::now()
                                      : cloud.header.stamp;
      bag.write(lidar_topic_, write_time, cloud);
    }
    bag.close();
    bag_status = "saved " +
                 std::to_string(diagnostic_cloud_buffer_.size()) +
                 " PointCloud2 frames";
  } catch (const std::exception& exception) {
    bag_status = std::string("failed: ") + exception.what();
  }

  std::string registration_map_status;
  try {
    const auto registration_map =
        trajLOdometry_->ExportRegistrationMap();
    if (!registration_map || registration_map->empty()) {
      registration_map_status = "not saved: registration map is empty";
    } else {
      const int status = pcl::io::savePCDFileBinaryCompressed(
          map_path.string(), *registration_map);
      registration_map_status =
          status == 0
              ? "saved " + std::to_string(registration_map->size()) +
                    " XYZ/intensity/label points"
              : "failed: PCL writer returned " + std::to_string(status);
    }
  } catch (const std::exception& exception) {
    registration_map_status =
        std::string("failed: ") + exception.what();
  }

  std::ofstream report(report_path.string(),
                       std::ios::out | std::ios::trunc);
  if (!report.is_open()) {
    ROS_ERROR("Failed to write diagnostic report '%s'",
              report_path.string().c_str());
    return;
  }

  report << std::setprecision(17)
         << "ONION_LO_FAILURE_ARTIFACTS_V1\n"
         << "reason=" << reason << "\n"
         << "lidar_topic=" << lidar_topic_ << "\n"
         << "message_stamp_sec=" << message.header.stamp.toSec() << "\n"
         << "message_stamp_ns=" << message.header.stamp.toNSec() << "\n"
         << "message_frame_id=" << message.header.frame_id << "\n"
         << "message_width=" << message.width << "\n"
         << "message_height=" << message.height << "\n"
         << "message_point_count="
         << static_cast<std::size_t>(message.width) * message.height << "\n"
         << "message_point_step=" << message.point_step << "\n"
         << "message_row_step=" << message.row_step << "\n"
         << "message_data_bytes=" << message.data.size() << "\n"
         << "message_is_bigendian="
         << (message.is_bigendian ? "true" : "false") << "\n"
         << "message_is_dense="
         << (message.is_dense ? "true" : "false") << "\n"
         << "diagnostic_point_layout_valid="
         << (can_inspect_points ? "true" : "false") << "\n"
         << "diagnostic_finite_points=" << finite_point_count << "\n";
  if (finite_point_count > 0) {
    report << "raw_cloud_minimum_xyz=" << raw_minimum.transpose() << "\n"
           << "raw_cloud_maximum_xyz=" << raw_maximum.transpose() << "\n"
           << "raw_cloud_extents_xyz="
           << (raw_maximum - raw_minimum).transpose() << "\n"
           << "point_time_min_sec=" << minimum_point_time << "\n"
           << "point_time_max_sec=" << maximum_point_time << "\n"
           << "point_time_span_ms="
           << (maximum_point_time - minimum_point_time) * 1e3 << "\n"
           << "point_time_min_to_header_ms="
           << (minimum_point_time - message.header.stamp.toSec()) * 1e3
           << "\n";
  }
  report
         << "point_fields_begin\n";
  for (const auto& field : message.fields) {
    report << "field name=" << field.name
           << " offset=" << field.offset
           << " datatype=" << static_cast<int>(field.datatype)
           << " count=" << field.count << "\n";
  }
  report << "point_fields_end\n"
         << "point_time_field=" << config_.point_time_field << "\n"
         << "point_time_scale=" << config_.point_time_scale << "\n"
         << "point_time_is_offset="
         << (config_.point_time_is_offset ? "true" : "false") << "\n"
         << "voxel_size=" << config_.voxel_size << "\n"
         << "max_points_per_voxel="
         << config_.max_points_per_voxel << "\n"
         << "max_iterations=" << config_.max_iterations << "\n"
         << "raw_point_num=" << config_.raw_point_num << "\n"
         << "min_registration_inliers="
         << config_.min_registration_inliers << "\n"
         << "optimizer_damping=" << config_.optimizer_damping << "\n"
         << "max_optimizer_translation_increment="
         << config_.max_optimizer_translation_increment << "\n"
         << "max_optimizer_rotation_increment_deg="
         << config_.max_optimizer_rotation_increment_deg << "\n"
         << "max_optimizer_translation_deviation="
         << config_.max_optimizer_translation_deviation << "\n"
         << "max_optimizer_rotation_deviation_deg="
         << config_.max_optimizer_rotation_deviation_deg << "\n"
         << "diagnostic_buffer_frames="
         << diagnostic_cloud_buffer_.size() << "\n";
  for (std::size_t index = 0;
       index < diagnostic_cloud_buffer_.size(); ++index) {
    const auto& cloud = diagnostic_cloud_buffer_[index];
    report << "buffer_frame index=" << index
           << " stamp_ns=" << cloud.header.stamp.toNSec()
           << " points="
           << static_cast<std::size_t>(cloud.width) * cloud.height
           << " bytes=" << cloud.data.size() << "\n";
  }

  const Sophus::SE3d pose = trajLOdometry_->current_pose;
  const Eigen::Quaterniond quaternion = pose.unit_quaternion();
  report << "last_safe_pose_translation="
         << pose.translation().transpose() << "\n"
         << "last_safe_pose_quaternion_xyzw="
         << quaternion.x() << " " << quaternion.y() << " "
         << quaternion.z() << " " << quaternion.w() << "\n"
         << "published_path_pose_count=" << path_msg_.poses.size() << "\n";
  const std::size_t trajectory_begin =
      path_msg_.poses.size() > 50 ? path_msg_.poses.size() - 50 : 0;
  for (std::size_t index = trajectory_begin;
       index < path_msg_.poses.size(); ++index) {
    const auto& path_pose = path_msg_.poses[index];
    report << "published_pose index=" << index
           << " stamp_ns=" << path_pose.header.stamp.toNSec()
           << " xyz=" << path_pose.pose.position.x << " "
           << path_pose.pose.position.y << " "
           << path_pose.pose.position.z
           << " quaternion_xyzw="
           << path_pose.pose.orientation.x << " "
           << path_pose.pose.orientation.y << " "
           << path_pose.pose.orientation.z << " "
           << path_pose.pose.orientation.w << "\n";
  }
  report
         << "context_bag=" << bag_path.string() << "\n"
         << "context_bag_status=" << bag_status << "\n"
         << "registration_map=" << map_path.string() << "\n"
         << "registration_map_status="
         << registration_map_status << "\n"
         << "odometry_report_begin\n"
         << trajLOdometry_->LastFailureReport()
         << "odometry_report_end\n";
  report.close();

  ROS_FATAL_STREAM(
      "Tracking failure artifacts saved. Report: "
      << report_path.string() << "; context bag: "
      << bag_path.string() << "; registration map: "
      << map_path.string());
}

traj::Scan::Ptr Onion_LO::Msg2Scan(const sensor_msgs::PointCloud2& msg) {
  traj::Scan::Ptr scan(new traj::Scan);
  scan->timestamp = msg.header.stamp.toNSec();
  const std::size_t point_count =
      static_cast<std::size_t>(msg.height) * msg.width;
  if (point_count == 0) {
    throw std::runtime_error("PointCloud2 contains no points");
  }
  if (msg.is_bigendian) {
    throw std::runtime_error(
        "big-endian PointCloud2 is not supported by the MID-360 adapter");
  }
  if (msg.point_step == 0 ||
      msg.row_step < msg.width * msg.point_step ||
      msg.data.size() <
          static_cast<std::size_t>(msg.row_step) * msg.height) {
    throw std::runtime_error("PointCloud2 has an invalid data layout");
  }

  const sensor_msgs::PointField* field_x = nullptr;
  const sensor_msgs::PointField* field_y = nullptr;
  const sensor_msgs::PointField* field_z = nullptr;
  const sensor_msgs::PointField* field_intensity = nullptr;
  const sensor_msgs::PointField* field_time = nullptr;
  for (const auto& field : msg.fields) {
    if (field.name == "x") field_x = &field;
    if (field.name == "y") field_y = &field;
    if (field.name == "z") field_z = &field;
    if (field.name == "intensity") field_intensity = &field;
    if (field.name == config_.point_time_field) field_time = &field;
  }

  const auto validate_field =
      [&](const sensor_msgs::PointField* field, const std::string& name,
          std::uint8_t datatype, std::size_t datatype_size) {
        if (!field) {
          throw std::runtime_error("PointCloud2 has no '" + name + "' field");
        }
        if (field->datatype != datatype || field->count < 1) {
          throw std::runtime_error("PointCloud2 field '" + name +
                                   "' has an unexpected datatype");
        }
        if (static_cast<std::size_t>(field->offset) + datatype_size >
            msg.point_step) {
          throw std::runtime_error("PointCloud2 field '" + name +
                                   "' exceeds point_step");
        }
      };

  validate_field(field_x, "x", sensor_msgs::PointField::FLOAT32,
                 sizeof(float));
  validate_field(field_y, "y", sensor_msgs::PointField::FLOAT32,
                 sizeof(float));
  validate_field(field_z, "z", sensor_msgs::PointField::FLOAT32,
                 sizeof(float));
  if (!field_time) {
    throw std::runtime_error("PointCloud2 has no '" +
                             config_.point_time_field + "' field");
  }
  validate_field(field_time, config_.point_time_field,
                 sensor_msgs::PointField::FLOAT64, sizeof(double));
  if (field_intensity) {
    validate_field(field_intensity, "intensity",
                   sensor_msgs::PointField::FLOAT32, sizeof(float));
  }

  if (!std::isfinite(config_.point_time_scale) ||
      config_.point_time_scale <= 0.0) {
    throw std::runtime_error("Onion/point_time_scale must be positive");
  }

  const double header_time = msg.header.stamp.toSec();
  double min_point_time = std::numeric_limits<double>::infinity();
  double max_point_time = -std::numeric_limits<double>::infinity();
  scan->points.reserve(point_count);

  for (std::uint32_t row = 0; row < msg.height; ++row) {
    const std::uint8_t* row_data =
        msg.data.data() + static_cast<std::size_t>(row) * msg.row_step;
    for (std::uint32_t column = 0; column < msg.width; ++column) {
      const std::uint8_t* point_data =
          row_data + static_cast<std::size_t>(column) * msg.point_step;

      traj::PointXYZIT point;
      point.x = ReadUnalignedField<float>(point_data, field_x->offset);
      point.y = ReadUnalignedField<float>(point_data, field_y->offset);
      point.z = ReadUnalignedField<float>(point_data, field_z->offset);
      const double raw_time =
          ReadUnalignedField<double>(point_data, field_time->offset);
      const double scaled_time = raw_time * config_.point_time_scale;
      point.ts = config_.point_time_is_offset
                     ? header_time + scaled_time
                     : scaled_time;
      point.label = 0.0;
      if (field_intensity) {
        point.intensity =
            ReadUnalignedField<float>(point_data, field_intensity->offset);
        if (!std::isfinite(point.intensity)) point.intensity = 0.0F;
      }

      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.ts)) {
        continue;
      }
      min_point_time = std::min(min_point_time, point.ts);
      max_point_time = std::max(max_point_time, point.ts);
      scan->points.emplace_back(point);
    }
  }

  if (scan->points.empty()) {
    throw std::runtime_error("PointCloud2 contains no finite points");
  }
  scan->size = scan->points.size();

  // A wrong absolute/offset setting otherwise fails silently much later in
  // PointCloudSegment. MID-360 frames at 10 Hz should be within 0.1 s; 5 s
  // leaves ample margin while still detecting epoch-time double counting.
  if (!msg.header.stamp.isZero() &&
      (std::abs(min_point_time - header_time) > 5.0 ||
       std::abs(max_point_time - header_time) > 5.0)) {
    throw std::runtime_error(
        "per-point timestamps are not close to header.stamp; check "
        "Onion/point_time_scale and Onion/point_time_is_offset");
  }
  if (max_point_time - min_point_time > 5.0) {
    throw std::runtime_error(
        "PointCloud2 per-point timestamp span exceeds 5 seconds");
  }

  ROS_INFO_STREAM_ONCE(
      "MID-360 PointCloud2 adapter verified: point_step=" << msg.point_step
      << ", timestamp_offset=" << field_time->offset
      << ", timestamp_mode="
      << (config_.point_time_is_offset ? "offset" : "absolute")
      << ", first_to_header_ms="
      << (min_point_time - header_time) * 1e3
      << ", frame_span_ms="
      << (max_point_time - min_point_time) * 1e3);

  if (scan->timestamp == 0) {
    scan->timestamp =
        static_cast<int64_t>(std::llround(min_point_time * 1e9));
  }
  return scan;
}

void Onion_LO::LiDAR_odom(
    const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
  const auto begin = std::chrono::high_resolution_clock::now();
  const ros::Time output_stamp =
      lidar_msg->header.stamp.isZero() ? ros::Time::now()
                                       : lidar_msg->header.stamp;
  save_timestamp_ = output_stamp.toSec();

  const auto curr_scan = Msg2Scan(*lidar_msg);
  trajLOdometry_->Start(curr_scan);

  auto deskew_scan = std::move(trajLOdometry_->deskew_points);
  trajLOdometry_->deskew_points.clear();
  const Sophus::SE3d new_pose = trajLOdometry_->current_pose;

  bool persistent_map_updated = false;
  if (!localization_mode_ && save_pcd_en_ && !deskew_scan.empty()) {
    std::string rejection_reason;
    if (ShouldAccumulateMap(new_pose, output_stamp, &rejection_reason)) {
      global_map_cache_.AddPoints(deskew_scan);
      persistent_map_updated = true;
    } else {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Persistent map skipped current scan: " << rejection_reason);
      if (map_integrity_fault_ && stop_on_tracking_failure_) {
        throw std::runtime_error(
            "persistent-map integrity failure: " + rejection_reason);
      }
    }
  }

  if (persistent_map_updated && publish_global_map_ &&
      ((scan_num_ + 1) % global_map_publish_interval_ == 0)) {
    PublishMappingGlobalMap(output_stamp);
  }

  Vector3dVector cloud_xyz;
  const PointCloudXYZRGB::Ptr display_cloud =
      convertToPCL(deskew_scan, cloud_xyz);

  const auto end = std::chrono::high_resolution_clock::now();
  const double duration_ms =
      std::chrono::duration<double, std::milli>(end - begin).count();
  total_duration_ms_ += duration_ms;
  const double average_ms =
      total_duration_ms_ / static_cast<double>(scan_num_ + 1);
  ROS_INFO_THROTTLE(1.0, "Onion-LO++ processing: current %.2f ms, avg %.2f ms",
                    duration_ms, average_ms);

  if (publish_octomap_ && !cloud_xyz.empty()) {
    octomap_cache_.XYZ_AddPoints(cloud_xyz);
    const Vector3dVector new_points = octomap_cache_.Pointcloud();
    for (const auto& point : new_points) {
      complete_map_->push_back(
          pcl::PointXYZ(point.x(), point.y(), point.z()));
    }

    octomap::OcTree tree(0.2);
    for (const auto& point : complete_map_->points) {
      tree.updateNode(
          octomap::point3d(point.x, point.y, point.z), true);
    }
    tree.updateInnerOccupancy();
    octomap_msgs::Octomap octomap_message;
    octomap_msgs::binaryMapToMsg(tree, octomap_message);
    octomap_message.header.stamp = output_stamp;
    octomap_message.header.frame_id = odom_frame_;
    octomap_pub_.publish(octomap_message);
    complete_map_->clear();
  }

  const Eigen::Vector3d translation = new_pose.translation();
  const Eigen::Quaterniond quaternion = new_pose.unit_quaternion();

  if (metrics_output_.is_open()) {
    const double wall_now_sec = ros::WallTime::now().toSec();
    if (!metrics_timing_initialized_) {
      metrics_timing_initialized_ = true;
      metrics_wall_start_sec_ = wall_now_sec;
      metrics_first_stamp_ = output_stamp;
      metrics_previous_stamp_ = output_stamp;
    }
    const double input_delta_ms =
        (output_stamp - metrics_previous_stamp_).toSec() * 1e3;
    const double sensor_elapsed_sec =
        (output_stamp - metrics_first_stamp_).toSec();
    const double wall_elapsed_sec =
        wall_now_sec - metrics_wall_start_sec_;
    metrics_output_
        << metrics_reset_id_ << ","
        << scan_num_ << ","
        << std::fixed << std::setprecision(9)
        << output_stamp.toSec() << ","
        << sensor_elapsed_sec << ","
        << wall_elapsed_sec << ","
        << input_delta_ms << ","
        << duration_ms << ","
        << average_ms << ","
        << trajLOdometry_->LastRegistrationInliers() << ","
        << trajLOdometry_->RegistrationMapPointCount() << ","
        << trajLOdometry_->RegistrationMapVoxelCount() << ","
        << translation.x() << ","
        << translation.y() << ","
        << translation.z() << ","
        << quaternion.x() << ","
        << quaternion.y() << ","
        << quaternion.z() << ","
        << quaternion.w() << "\n";
    metrics_output_.flush();
    metrics_previous_stamp_ = output_stamp;
  }

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = output_stamp;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = child_frame_;
  transform.transform.translation.x = translation.x();
  transform.transform.translation.y = translation.y();
  transform.transform.translation.z = translation.z();
  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
  tf_broadcaster_.sendTransform(transform);

  nav_msgs::Odometry odometry;
  odometry.header = transform.header;
  odometry.child_frame_id = child_frame_;
  odometry.pose.pose.position.x = translation.x();
  odometry.pose.pose.position.y = translation.y();
  odometry.pose.pose.position.z = translation.z();
  odometry.pose.pose.orientation = transform.transform.rotation;
  odom_publisher_.publish(odometry);

  geometry_msgs::PoseStamped pose;
  pose.header = odometry.header;
  pose.pose = odometry.pose.pose;
  pose_publisher_.publish(pose);

  path_msg_.header = pose.header;
  path_msg_.poses.emplace_back(pose);
  if (path_msg_.poses.size() > path_max_size_) {
    const auto remove_count = path_msg_.poses.size() - path_max_size_;
    path_msg_.poses.erase(path_msg_.poses.begin(),
                          path_msg_.poses.begin() + remove_count);
  }
  traj_publisher_.publish(path_msg_);

  ROS_INFO_STREAM_THROTTLE(
      1.0, std::fixed << std::setprecision(3)
                      << "Pose [" << odom_frame_ << " -> " << child_frame_
                      << "] xyz=(" << translation.x() << ", "
                      << translation.y() << ", " << translation.z()
                      << ") q=(" << quaternion.x() << ", "
                      << quaternion.y() << ", " << quaternion.z() << ", "
                      << quaternion.w() << ")");

  if (save_path_) {
    const fs::path trajectory_path =
        fs::path(ROOT_DIR) / "results" / "Onion.txt";
    boost::system::error_code error_code;
    fs::create_directories(trajectory_path.parent_path(), error_code);
    std::ofstream output(trajectory_path.string(), std::ios::app);
    output << std::fixed << std::setprecision(9)
           << save_timestamp_ << " " << translation.x() << " "
           << translation.y() << " " << translation.z() << " "
           << quaternion.x() << " " << quaternion.y() << " "
           << quaternion.z() << " " << quaternion.w() << "\n";
  }

  sensor_msgs::PointCloud2 cloud_message;
  pcl::toROSMsg(*display_cloud, cloud_message);
  cloud_message.header.stamp = output_stamp;
  cloud_message.header.frame_id = odom_frame_;
  if (local_map_publisher_.getNumSubscribers() > 0) {
    local_map_publisher_.publish(cloud_message);
  }
  frame_publisher_.publish(cloud_message);

  ++scan_num_;
}

bool Onion_LO::ShouldAccumulateMap(const Sophus::SE3d& pose,
                                   const ros::Time& stamp,
                                   std::string* reason) {
  if (map_integrity_fault_) {
    if (reason) {
      *reason =
          "map integrity protection is latched; restart mapping after "
          "checking odometry. First fault: " +
          map_integrity_fault_reason_;
    }
    return false;
  }
  if (!pose.matrix().allFinite()) {
    map_integrity_fault_ = true;
    map_integrity_fault_reason_ =
        "odometry pose contains NaN or infinity";
    if (reason) *reason = map_integrity_fault_reason_;
    return false;
  }

  bool motion_is_valid = true;
  std::string motion_error;
  if (previous_mapping_pose_valid_) {
    const double delta_time = (stamp - previous_mapping_stamp_).toSec();
    if (delta_time <= 1e-6) {
      motion_is_valid = false;
      motion_error = "PointCloud2 timestamps are not strictly increasing";
    } else {
      const Sophus::SE3d delta = previous_mapping_pose_.inverse() * pose;
      const double linear_speed = delta.translation().norm() / delta_time;
      const double angular_speed_deg =
          delta.so3().log().norm() * 180.0 /
          3.14159265358979323846 / delta_time;
      if (!std::isfinite(linear_speed) ||
          !std::isfinite(angular_speed_deg) ||
          linear_speed > max_mapping_linear_speed_ ||
          angular_speed_deg > max_mapping_angular_speed_deg_) {
        motion_is_valid = false;
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2)
               << "implausible odometry motion (linear=" << linear_speed
               << " m/s, angular=" << angular_speed_deg
               << " deg/s); limits are " << max_mapping_linear_speed_
               << " m/s and " << max_mapping_angular_speed_deg_ << " deg/s";
        motion_error = stream.str();
      }
    }
  }

  previous_mapping_pose_ = pose;
  previous_mapping_stamp_ = stamp;
  previous_mapping_pose_valid_ = true;

  if (!motion_is_valid) {
    map_integrity_fault_ = true;
    map_integrity_fault_reason_ = motion_error;
    if (reason) *reason = map_integrity_fault_reason_;
    return false;
  }
  if (!trajLOdometry_->IsTrackingHealthy()) {
    map_integrity_fault_ = true;
    std::ostringstream stream;
    stream << "registration inliers "
           << trajLOdometry_->LastRegistrationInliers()
           << " are below Traj/min_registration_inliers="
           << config_.min_registration_inliers;
    map_integrity_fault_reason_ = stream.str();
    if (reason) *reason = map_integrity_fault_reason_;
    return false;
  }
  return true;
}

void Onion_LO::PublishMappingGlobalMap(const ros::Time& stamp) {
  const auto cloud = global_map_cache_.Snapshot();
  if (cloud->empty()) return;

  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  message.header.stamp = stamp;
  message.header.frame_id = odom_frame_;
  global_map_publisher_.publish(message);
}

PointCloudXYZRGB::Ptr Onion_LO::convertToPCL(
    const std::vector<traj::PointXYZI>& input, Vector3dVector& cloud_xyz) {
  PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB());
  cloud->points.reserve(input.size());
  cloud_xyz.clear();
  cloud_xyz.reserve(input.size());

  for (const auto& point : input) {
    pcl::PointXYZRGB output;
    output.x = point.x;
    output.y = point.y;
    output.z = point.z;

    const int label = static_cast<int>(std::lround(point.label));
    if (label == 0) {
      output.r = 0;
      output.g = 100;
      output.b = 130;
    } else if (label == 1) {
      output.r = 255;
      output.g = 255;
      output.b = 0;
    } else {
      output.r = 255;
      output.g = 0;
      output.b = 0;
    }
    cloud->points.emplace_back(output);
    cloud_xyz.emplace_back(point.x, point.y, point.z);
  }

  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

bool Onion_LO::SaveGlobalMap(std::string* message) {
  if (localization_mode_) {
    if (message) {
      *message = "node is in localization mode; loaded map is read-only";
    }
    return false;
  }
  if (!save_pcd_en_) {
    if (message) *message = "Map/save_pcd_en is false";
    return false;
  }
  if (map_integrity_fault_) {
    if (message) {
      *message =
          "map was not saved because odometry integrity protection was "
          "triggered: " + map_integrity_fault_reason_ +
          "; fix the cause and remap";
    }
    return false;
  }

  const auto statistics = global_map_cache_.GetStatistics();
  if (statistics.point_count <
      static_cast<std::size_t>(minimum_map_save_points_)) {
    if (message) {
      *message = "map has only " + std::to_string(statistics.point_count) +
                 " points; minimum_map_save_points is " +
                 std::to_string(minimum_map_save_points_);
    }
    return false;
  }
  const Eigen::Vector3d extents = statistics.Extents();
  const double extent_ratio =
      statistics.SecondaryToPrimaryExtentRatio();
  if (reject_line_like_map_ && extents.maxCoeff() > 1.0 &&
      extent_ratio < minimum_secondary_extent_ratio_) {
    if (message) {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3)
             << "map was not saved because its bounds look line-like: "
             << "extents=(" << extents.x() << ", " << extents.y() << ", "
             << extents.z() << ") m, secondary/primary=" << extent_ratio
             << "; check odometry and timestamp configuration";
      *message = stream.str();
    }
    return false;
  }

  const fs::path output_path(map_path_);
  boost::system::error_code error_code;
  if (!output_path.parent_path().empty()) {
    fs::create_directories(output_path.parent_path(), error_code);
  }
  if (error_code) {
    if (message) {
      *message = "failed to create map directory: " + error_code.message();
    }
    return false;
  }

  std::size_t saved_points = 0;
  std::string error;
  if (!global_map_cache_.SavePCD(map_path_, map_binary_compressed_,
                                &saved_points, &error)) {
    if (message) *message = error;
    return false;
  }

  if (message) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "saved " << saved_points
           << " centroid-filtered XYZ/intensity/label points to "
           << map_path_
           << (map_binary_compressed_ ? " (binary_compressed)"
                                      : " (binary)")
           << "; extents=(" << extents.x() << ", " << extents.y() << ", "
           << extents.z() << ") m";
    *message = stream.str();
  }
  return true;
}

bool Onion_LO::SaveMapService(std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& response) {
  response.success = SaveGlobalMap(&response.message);
  if (response.success) {
    ROS_INFO_STREAM(response.message);
  } else {
    ROS_WARN_STREAM(response.message);
  }
  return true;
}

void Onion_LO::InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  Eigen::Quaterniond quaternion(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  if (quaternion.norm() < 1e-6) {
    ROS_ERROR("Rejected /initialpose with an invalid quaternion");
    return;
  }
  quaternion.normalize();
  const Eigen::Vector3d translation(
      msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);

  if (trajLOdometry_->SetInitialPose(
          Sophus::SE3d(quaternion.toRotationMatrix(), translation))) {
    path_msg_.poses.clear();
    metrics_timing_initialized_ = false;
    ++metrics_reset_id_;
    ROS_INFO("Localization state reset from /initialpose");
  } else {
    ROS_WARN("/initialpose is only accepted in localization mode");
  }
}

void Onion_LO::PublishLoadedMap() {
  const auto cloud = trajLOdometry_->ExportRegistrationMap();
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  message.header.stamp = ros::Time::now();
  message.header.frame_id = odom_frame_;
  global_map_publisher_.publish(message);
  ROS_INFO("Published loaded global map with %zu points", cloud->size());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Onion_LO_main");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  try {
    Onion_LO onion_lo(nh, private_nh);
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("Failed to initialize Onion-LO++: %s", exception.what());
    return 1;
  }
  return 0;
}
