#pragma once

#include <chrono>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ros_common_lib.h>
#include <trajlo/common_type.h>
#include <trajlo/map_point_type.hpp>

#include "Save_Map.hpp"
#include "traj_odom.hpp"

class Onion_LO {
 public:
  Onion_LO(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~Onion_LO();

 private:
  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void LiDAR_odom(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);
  traj::Scan::Ptr Msg2Scan(const sensor_msgs::PointCloud2& msg);
  PointCloudXYZRGB::Ptr convertToPCL(
      const std::vector<traj::PointXYZI>& input, Vector3dVector& cloud_xyz);

  bool SaveMapService(std_srvs::Trigger::Request& request,
                      std_srvs::Trigger::Response& response);
  bool SaveGlobalMap(std::string* message = nullptr);
  bool ShouldAccumulateMap(const Sophus::SE3d& pose,
                           const ros::Time& stamp,
                           std::string* reason);
  void PublishMappingGlobalMap(const ros::Time& stamp);
  void BufferDiagnosticFrame(
      const sensor_msgs::PointCloud2::ConstPtr& message);
  void DumpTrackingFailure(const sensor_msgs::PointCloud2& message,
                           const std::string& reason);
  void InitialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void PublishLoadedMap();
  std::string ResolveMapPath(const std::string& configured_path) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_lidar_;
  ros::Subscriber initial_pose_subscriber_;
  ros::Publisher odom_publisher_;
  ros::Publisher pose_publisher_;
  ros::Publisher traj_publisher_;
  ros::Publisher frame_publisher_;
  ros::Publisher local_map_publisher_;
  ros::Publisher global_map_publisher_;
  ros::Publisher octomap_pub_;
  ros::ServiceServer save_map_server_;

  nav_msgs::Path path_msg_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  int queue_size_ = 1;
  int scan_num_ = 0;
  double save_timestamp_ = 0.0;
  double total_duration_ms_ = 0.0;

  std::string odom_frame_ = "odom";
  std::string child_frame_ = "base_link";
  std::string lidar_topic_ = "/livox/lidar";
  std::string map_path_;

  bool save_path_ = false;
  bool publish_octomap_ = false;
  bool save_pcd_en_ = true;
  bool save_map_on_shutdown_ = true;
  bool map_binary_compressed_ = true;
  bool localization_mode_ = false;
  bool publish_global_map_ = true;
  bool reject_line_like_map_ = true;
  double global_map_voxel_size_ = 0.10;
  double minimum_secondary_extent_ratio_ = 0.02;
  double max_mapping_linear_speed_ = 3.0;
  double max_mapping_angular_speed_deg_ = 240.0;
  int global_map_publish_interval_ = 20;
  int minimum_map_save_points_ = 1000;
  std::size_t path_max_size_ = 5000;
  bool previous_mapping_pose_valid_ = false;
  bool map_integrity_fault_ = false;
  std::string map_integrity_fault_reason_;
  bool diagnostics_enabled_ = true;
  bool stop_on_tracking_failure_ = true;
  bool diagnostic_failure_dumped_ = false;
  int diagnostic_context_frames_ = 10;
  std::string diagnostic_output_directory_;
  std::deque<sensor_msgs::PointCloud2> diagnostic_cloud_buffer_;
  Sophus::SE3d previous_mapping_pose_;
  ros::Time previous_mapping_stamp_;

  PointCloudXYZ::Ptr complete_map_;
  Save_VoxelHashMap octomap_cache_;
  GlobalMapVoxelCache global_map_cache_;
  traj::TrajConfig config_;
  traj::TrajLOdometry::Ptr trajLOdometry_;
};
