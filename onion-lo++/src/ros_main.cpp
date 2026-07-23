#include "ros_main.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

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
  pnh_.param("Map/child_frame", child_frame_, std::string("base_link"));
  pnh_.param("Map/odom_frame", odom_frame_, std::string("odom"));
  pnh_.param("Map/localization_mode", localization_mode_, false);
  pnh_.param("Map/update_loaded_map", config_.update_loaded_map, false);
  pnh_.param("Map/loaded_map_max_points_per_voxel",
             config_.loaded_map_max_points_per_voxel, 0);

  int path_max_size = 5000;
  pnh_.param("Map/path_max_size", path_max_size, 5000);
  path_max_size_ = static_cast<std::size_t>(std::max(1, path_max_size));

  std::string configured_map_path;
  pnh_.param("Map/map_path", configured_map_path,
             std::string("results/onion_map.pcd"));
  map_path_ = ResolveMapPath(configured_map_path);

  pnh_.param("Onion/lidar_topic", lidar_topic_,
             std::string("/livox/lidar"));
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

  global_map_cache_.Configure(global_map_voxel_size_);
  complete_map_.reset(new PointCloudXYZ());
  octomap_cache_.voxel_size_ = 2.0;
  octomap_cache_.reslt_ = 0.2;

  odom_publisher_ =
      pnh_.advertise<nav_msgs::Odometry>("odometry", queue_size_);
  pose_publisher_ =
      pnh_.advertise<geometry_msgs::PoseStamped>("pose", queue_size_);
  frame_publisher_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("frame", queue_size_);
  local_map_publisher_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("local_map", queue_size_);
  global_map_publisher_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
  octomap_pub_ =
      pnh_.advertise<octomap_msgs::Octomap>("octomap_binary", queue_size_);
  traj_publisher_ =
      pnh_.advertise<nav_msgs::Path>("trajectory", queue_size_);
  path_msg_.header.frame_id = odom_frame_;

  trajLOdometry_.reset(new traj::TrajLOdometry(config_));

  sub_lidar_ = nh_.subscribe(lidar_topic_, queue_size_,
                             &Onion_LO::PointCloudCallback, this,
                             ros::TransportHints().tcpNoDelay());
  save_map_server_ =
      pnh_.advertiseService("save_map", &Onion_LO::SaveMapService, this);

  if (localization_mode_) {
    initial_pose_subscriber_ =
        nh_.subscribe("/initialpose", 1, &Onion_LO::InitialPoseCallback, this);
    PublishLoadedMap();
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
  try {
    LiDAR_odom(msg);
  } catch (const std::exception& exception) {
    ROS_ERROR_THROTTLE(1.0, "PointCloud2 conversion/odometry failed: %s",
                       exception.what());
  }
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

  if (!localization_mode_ && save_pcd_en_ && !deskew_scan.empty()) {
    global_map_cache_.AddPoints(deskew_scan);
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
  local_map_publisher_.publish(cloud_message);
  frame_publisher_.publish(cloud_message);

  ++scan_num_;
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
    *message = "saved " + std::to_string(saved_points) +
               " XYZ/intensity/label points to " + map_path_ +
               (map_binary_compressed_ ? " (binary_compressed)"
                                       : " (binary)");
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
