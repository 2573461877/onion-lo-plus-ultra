#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

namespace onion_global_relocalization {

class RelocalizationAdapter {
 public:
  RelocalizationAdapter()
      : nh_(), private_nh_("~"), last_attempt_(0.0) {
    private_nh_.param("map_path", map_path_, std::string());
    private_nh_.param("map_frame", map_frame_, std::string("odom"));
    private_nh_.param("lidar_topic", lidar_topic_,
                      std::string("/livox/lidar"));
    private_nh_.param("set_map_service", set_map_service_,
                      std::string("/hdl_global_localization/set_global_map"));
    private_nh_.param("query_service", query_service_,
                      std::string("/hdl_global_localization/query"));
    private_nh_.param("initialpose_topic", initialpose_topic_,
                      std::string("/initialpose"));
    private_nh_.param("auto_relocalize", auto_relocalize_, true);
    private_nh_.param("max_num_candidates", max_num_candidates_, 3);
    private_nh_.param("max_query_attempts", max_query_attempts_, 3);
    private_nh_.param("retry_delay_sec", retry_delay_sec_, 1.0);
    private_nh_.param("min_inlier_fraction", min_inlier_fraction_, 0.25);
    private_nh_.param("max_matching_error", max_matching_error_, -1.0);
    private_nh_.param("service_wait_timeout_sec",
                      service_wait_timeout_sec_, 30.0);
    private_nh_.param("position_stddev", position_stddev_, 0.5);
    private_nh_.param("orientation_stddev", orientation_stddev_, 0.35);

    ValidateParameters();

    initialpose_publisher_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            initialpose_topic_, 1, true);
    candidate_publisher_ =
        private_nh_.advertise<geometry_msgs::PoseStamped>(
            "candidate_pose", 1, true);
    trigger_server_ =
        private_nh_.advertiseService(
            "relocalize", &RelocalizationAdapter::Trigger, this);

    set_map_client_ =
        nh_.serviceClient<hdl_global_localization::SetGlobalMap>(
            set_map_service_);
    query_client_ =
        nh_.serviceClient<hdl_global_localization::QueryGlobalLocalization>(
            query_service_);

    WaitForService(set_map_client_, set_map_service_);
    WaitForService(query_client_, query_service_);
    LoadAndSetGlobalMap();

    lidar_subscriber_ =
        nh_.subscribe(lidar_topic_, 1,
                      &RelocalizationAdapter::PointCloudCallback, this,
                      ros::TransportHints().tcpNoDelay());
    relocalization_pending_ = auto_relocalize_;

    ROS_INFO(
        "Onion global relocalization adapter ready: map='%s', "
        "LiDAR='%s', auto=%s",
        map_path_.c_str(), lidar_topic_.c_str(),
        auto_relocalize_ ? "true" : "false");
  }

 private:
  void ValidateParameters() const {
    if (map_path_.empty()) {
      throw std::runtime_error("~map_path must not be empty");
    }
    if (map_frame_.empty() || lidar_topic_.empty() ||
        set_map_service_.empty() || query_service_.empty() ||
        initialpose_topic_.empty()) {
      throw std::runtime_error(
          "frame, topic, and service parameters must not be empty");
    }
    if (max_num_candidates_ <= 0 || max_query_attempts_ <= 0) {
      throw std::runtime_error(
          "~max_num_candidates and ~max_query_attempts must be positive");
    }
    if (retry_delay_sec_ < 0.0 || service_wait_timeout_sec_ <= 0.0) {
      throw std::runtime_error(
          "retry delay must be non-negative and service timeout positive");
    }
    if (min_inlier_fraction_ < 0.0 || min_inlier_fraction_ > 1.0) {
      throw std::runtime_error("~min_inlier_fraction must be in [0, 1]");
    }
    if (position_stddev_ <= 0.0 || orientation_stddev_ <= 0.0) {
      throw std::runtime_error("initial-pose standard deviations must be positive");
    }
  }

  void WaitForService(ros::ServiceClient& client,
                      const std::string& service_name) {
    ROS_INFO("Waiting for service %s", service_name.c_str());
    if (!client.waitForExistence(
            ros::Duration(service_wait_timeout_sec_))) {
      throw std::runtime_error(
          "timed out waiting for service: " + service_name);
    }
  }

  void LoadAndSetGlobalMap() {
    pcl::PointCloud<pcl::PointXYZ> global_map;
    if (pcl::io::loadPCDFile(map_path_, global_map) != 0) {
      throw std::runtime_error("failed to load global PCD: " + map_path_);
    }

    pcl::PointCloud<pcl::PointXYZ> finite_map;
    std::vector<int> retained_indices;
    pcl::removeNaNFromPointCloud(
        global_map, finite_map, retained_indices);
    if (finite_map.empty()) {
      throw std::runtime_error(
          "global PCD contains no finite XYZ points: " + map_path_);
    }

    hdl_global_localization::SetGlobalMap service;
    pcl::toROSMsg(finite_map, service.request.global_map);
    service.request.global_map.header.frame_id = map_frame_;
    service.request.global_map.header.stamp = ros::Time(0);

    ROS_INFO("Sending %zu global map points to %s",
             finite_map.size(), set_map_service_.c_str());
    if (!set_map_client_.call(service)) {
      throw std::runtime_error(
          "failed to send global map through service: " +
          set_map_service_);
    }
    ROS_INFO("HDL global map initialization completed");
  }

  bool Trigger(std_srvs::Trigger::Request&,
               std_srvs::Trigger::Response& response) {
    relocalization_pending_ = true;
    attempts_ = 0;
    last_attempt_ = ros::WallTime(0.0);
    response.success = true;
    response.message =
        "relocalization armed; the next LiDAR frame will be queried";
    ROS_INFO("%s", response.message.c_str());
    return true;
  }

  void PointCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    if (!relocalization_pending_) return;
    if ((ros::WallTime::now() - last_attempt_).toSec() <
        retry_delay_sec_) {
      return;
    }

    last_attempt_ = ros::WallTime::now();
    ++attempts_;
    ROS_INFO("Global relocalization query attempt %d/%d with %u points",
             attempts_, max_query_attempts_,
             cloud->width * cloud->height);

    hdl_global_localization::QueryGlobalLocalization service;
    service.request.max_num_candidates = max_num_candidates_;
    service.request.cloud = *cloud;
    if (!query_client_.call(service)) {
      HandleFailure("HDL query service returned no solution");
      return;
    }

    const std::size_t candidate_count =
        std::min(service.response.poses.size(),
                 std::min(service.response.inlier_fractions.size(),
                          service.response.errors.size()));
    if (candidate_count == 0) {
      HandleFailure("HDL query returned empty candidate arrays");
      return;
    }

    std::size_t best_index = 0;
    for (std::size_t i = 1; i < candidate_count; ++i) {
      if (service.response.inlier_fractions[i] >
              service.response.inlier_fractions[best_index] ||
          (service.response.inlier_fractions[i] ==
               service.response.inlier_fractions[best_index] &&
           service.response.errors[i] <
               service.response.errors[best_index])) {
        best_index = i;
      }
    }

    const double inlier_fraction =
        service.response.inlier_fractions[best_index];
    const double matching_error = service.response.errors[best_index];
    const geometry_msgs::Pose& pose =
        service.response.poses[best_index];
    if (!std::isfinite(inlier_fraction) ||
        !std::isfinite(matching_error) ||
        inlier_fraction < min_inlier_fraction_ ||
        (max_matching_error_ >= 0.0 &&
         matching_error > max_matching_error_) ||
        !PoseIsFinite(pose)) {
      HandleFailure(
          "best HDL candidate failed confidence or finite-value checks");
      return;
    }

    const double quaternion_norm = std::sqrt(
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w);
    if (quaternion_norm < 1e-6) {
      HandleFailure("best HDL candidate has an invalid quaternion");
      return;
    }

    geometry_msgs::Pose normalized_pose = pose;
    normalized_pose.orientation.x /= quaternion_norm;
    normalized_pose.orientation.y /= quaternion_norm;
    normalized_pose.orientation.z /= quaternion_norm;
    normalized_pose.orientation.w /= quaternion_norm;
    PublishInitialPose(
        normalized_pose, cloud->header.stamp,
        inlier_fraction, matching_error);
    relocalization_pending_ = false;
  }

  bool PoseIsFinite(const geometry_msgs::Pose& pose) const {
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
  }

  void PublishInitialPose(const geometry_msgs::Pose& pose,
                          const ros::Time& stamp,
                          double inlier_fraction,
                          double matching_error) {
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.stamp =
        stamp.isZero() ? ros::Time::now() : stamp;
    initial_pose.header.frame_id = map_frame_;
    initial_pose.pose.pose = pose;

    const double position_variance =
        position_stddev_ * position_stddev_;
    const double orientation_variance =
        orientation_stddev_ * orientation_stddev_;
    initial_pose.pose.covariance[0] = position_variance;
    initial_pose.pose.covariance[7] = position_variance;
    initial_pose.pose.covariance[14] = position_variance;
    initial_pose.pose.covariance[21] = orientation_variance;
    initial_pose.pose.covariance[28] = orientation_variance;
    initial_pose.pose.covariance[35] = orientation_variance;
    initialpose_publisher_.publish(initial_pose);

    geometry_msgs::PoseStamped candidate;
    candidate.header = initial_pose.header;
    candidate.pose = pose;
    candidate_publisher_.publish(candidate);

    ROS_INFO(
        "Published HDL /initialpose: xyz=(%.3f, %.3f, %.3f), "
        "inlier_fraction=%.4f, error=%.4f",
        pose.position.x, pose.position.y, pose.position.z,
        inlier_fraction, matching_error);
  }

  void HandleFailure(const std::string& reason) {
    if (attempts_ >= max_query_attempts_) {
      relocalization_pending_ = false;
      ROS_ERROR(
          "Global relocalization stopped after %d attempts: %s. "
          "Call ~relocalize to try again.",
          attempts_, reason.c_str());
    } else {
      ROS_WARN(
          "Global relocalization attempt %d failed: %s; "
          "waiting for another LiDAR frame",
          attempts_, reason.c_str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber lidar_subscriber_;
  ros::Publisher initialpose_publisher_;
  ros::Publisher candidate_publisher_;
  ros::ServiceServer trigger_server_;
  ros::ServiceClient set_map_client_;
  ros::ServiceClient query_client_;

  std::string map_path_;
  std::string map_frame_;
  std::string lidar_topic_;
  std::string set_map_service_;
  std::string query_service_;
  std::string initialpose_topic_;
  bool auto_relocalize_ = true;
  bool relocalization_pending_ = false;
  int max_num_candidates_ = 3;
  int max_query_attempts_ = 3;
  int attempts_ = 0;
  double retry_delay_sec_ = 1.0;
  double min_inlier_fraction_ = 0.25;
  double max_matching_error_ = -1.0;
  double service_wait_timeout_sec_ = 30.0;
  double position_stddev_ = 0.5;
  double orientation_stddev_ = 0.35;
  ros::WallTime last_attempt_;
};

}  // namespace onion_global_relocalization

int main(int argc, char** argv) {
  ros::init(argc, argv, "onion_global_relocalization");
  try {
    onion_global_relocalization::RelocalizationAdapter adapter;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("Failed to initialize global relocalization adapter: %s",
              exception.what());
    return 1;
  }
  return 0;
}
