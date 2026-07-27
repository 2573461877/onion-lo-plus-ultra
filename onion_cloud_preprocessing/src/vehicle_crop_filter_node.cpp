#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "onion_cloud_preprocessing/pointcloud_crop.hpp"

namespace {

std::string NormalizeFrameId(const std::string& frame_id) {
  const auto first_character = frame_id.find_first_not_of('/');
  if (first_character == std::string::npos) return {};
  return frame_id.substr(first_character);
}

Eigen::Isometry3d ToEigen(
    const geometry_msgs::TransformStamped& transform) {
  const auto& rotation = transform.transform.rotation;
  Eigen::Quaterniond quaternion(
      rotation.w, rotation.x, rotation.y, rotation.z);
  if (!quaternion.coeffs().allFinite() ||
      quaternion.norm() < 1.0e-9) {
    throw std::runtime_error("TF contains an invalid quaternion");
  }
  quaternion.normalize();
  const auto& translation = transform.transform.translation;
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.linear() = quaternion.toRotationMatrix();
  output.translation() =
      Eigen::Vector3d(translation.x, translation.y, translation.z);
  if (!output.matrix().allFinite()) {
    throw std::runtime_error("TF contains a non-finite value");
  }
  return output;
}

class VehicleCropFilterNode {
 public:
  VehicleCropFilterNode()
      : private_node_("~"), transform_listener_(transform_buffer_) {
    private_node_.param<std::string>(
        "input_topic", input_topic_, "/fusion_rslidar_points");
    private_node_.param<std::string>(
        "filtered_sensor_topic", filtered_sensor_topic_,
        "/onion/points_filtered_sensor");
    private_node_.param<std::string>(
        "filtered_vehicle_topic", filtered_vehicle_topic_,
        "/onion/points_filtered_vehicle");
    private_node_.param<std::string>(
        "vehicle_frame", vehicle_frame_, "vehicle_link");
    private_node_.param("min_x", bounds_.min_x, -0.6);
    private_node_.param("max_x", bounds_.max_x, 1.65);
    private_node_.param("min_y", bounds_.min_y, -0.7);
    private_node_.param("max_y", bounds_.max_y, 0.7);
    private_node_.param("min_z", bounds_.min_z, -0.5);
    private_node_.param("max_z", bounds_.max_z, 2.5);
    private_node_.param("tf_timeout_sec", tf_timeout_sec_, 2.0);
    private_node_.param(
        "fail_if_tf_unavailable", fail_if_tf_unavailable_, true);
    private_node_.param("subscriber_queue_size",
                        subscriber_queue_size_, 10);
    private_node_.param("publisher_queue_size",
                        publisher_queue_size_, 10);

    vehicle_frame_ = NormalizeFrameId(vehicle_frame_);
    if (input_topic_.empty() || filtered_sensor_topic_.empty() ||
        filtered_vehicle_topic_.empty() || vehicle_frame_.empty()) {
      throw std::runtime_error(
          "vehicle crop topics and vehicle_frame must not be empty");
    }
    if (input_topic_ == filtered_sensor_topic_ ||
        input_topic_ == filtered_vehicle_topic_ ||
        filtered_sensor_topic_ == filtered_vehicle_topic_) {
      throw std::runtime_error(
          "vehicle crop input and output topics must be distinct");
    }
    if (!bounds_.valid() || !std::isfinite(tf_timeout_sec_) ||
        tf_timeout_sec_ < 0.0 || subscriber_queue_size_ <= 0 ||
        publisher_queue_size_ <= 0) {
      throw std::runtime_error(
          "invalid vehicle crop bounds, timeout, or queue size");
    }

    sensor_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
        filtered_sensor_topic_, publisher_queue_size_);
    vehicle_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
        filtered_vehicle_topic_, publisher_queue_size_);
    subscriber_ = node_.subscribe(
        input_topic_, subscriber_queue_size_,
        &VehicleCropFilterNode::CloudCallback, this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM(
        "Shared vehicle crop ready: input=" << input_topic_
        << ", sensor_output=" << filtered_sensor_topic_
        << ", vehicle_output=" << filtered_vehicle_topic_
        << ", vehicle_frame=" << vehicle_frame_
        << ", bounds=[" << bounds_.min_x << ", " << bounds_.max_x
        << "] x [" << bounds_.min_y << ", " << bounds_.max_y
        << "] x [" << bounds_.min_z << ", " << bounds_.max_z << "]");
  }

 private:
  bool LookupTransform(const std::string& source_frame,
                       Eigen::Isometry3d* vehicle_from_sensor) {
    if (source_frame == vehicle_frame_) {
      *vehicle_from_sensor = Eigen::Isometry3d::Identity();
      return true;
    }
    if (transform_cached_ && cached_source_frame_ == source_frame) {
      *vehicle_from_sensor = vehicle_from_sensor_;
      return true;
    }
    try {
      const auto transform = transform_buffer_.lookupTransform(
          vehicle_frame_, source_frame, ros::Time(0),
          ros::Duration(tf_timeout_sec_));
      vehicle_from_sensor_ = ToEigen(transform);
      cached_source_frame_ = source_frame;
      transform_cached_ = true;
      *vehicle_from_sensor = vehicle_from_sensor_;
      ROS_INFO_STREAM(
          "Shared vehicle crop cached TF " << vehicle_frame_
          << " <- " << source_frame);
      return true;
    } catch (const tf2::TransformException& error) {
      if (fail_if_tf_unavailable_) {
        ROS_FATAL_STREAM(
            "Shared vehicle crop cannot resolve TF " << vehicle_frame_
            << " <- " << source_frame << ": " << error.what());
        ros::shutdown();
      } else {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "Shared vehicle crop is waiting for TF "
                     << vehicle_frame_ << " <- " << source_frame
                     << ": " << error.what());
      }
      return false;
    }
  }

  void CloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& message) {
    const std::string source_frame =
        NormalizeFrameId(message->header.frame_id);
    if (source_frame.empty()) {
      ROS_ERROR_THROTTLE(
          1.0, "Shared vehicle crop rejected a cloud with no frame_id");
      return;
    }

    Eigen::Isometry3d vehicle_from_sensor =
        Eigen::Isometry3d::Identity();
    if (!LookupTransform(source_frame, &vehicle_from_sensor)) return;

    try {
      auto result = onion_cloud_preprocessing::CropAndTransform(
          *message, vehicle_from_sensor, bounds_, true, true);
      result.sensor_cloud.header.frame_id = source_frame;
      result.vehicle_cloud.header.frame_id = vehicle_frame_;
      sensor_publisher_.publish(result.sensor_cloud);
      vehicle_publisher_.publish(result.vehicle_cloud);

      cumulative_finite_points_ += result.finite_points;
      cumulative_removed_points_ += result.removed_vehicle_points;
      const double current_ratio =
          result.finite_points == 0
              ? 0.0
              : 100.0 * result.removed_vehicle_points /
                    static_cast<double>(result.finite_points);
      const double cumulative_ratio =
          cumulative_finite_points_ == 0
              ? 0.0
              : 100.0 * cumulative_removed_points_ /
                    static_cast<double>(cumulative_finite_points_);
      ROS_INFO_STREAM_THROTTLE(
          1.0, "Shared vehicle crop removed "
                   << result.removed_vehicle_points << "/"
                   << result.finite_points << " finite points ("
                   << current_ratio << "%), output="
                   << result.output_points << ", cumulative="
                   << cumulative_ratio << "%");
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Shared vehicle crop rejected PointCloud2: "
                   << error.what());
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener_;
  ros::Subscriber subscriber_;
  ros::Publisher sensor_publisher_;
  ros::Publisher vehicle_publisher_;

  std::string input_topic_;
  std::string filtered_sensor_topic_;
  std::string filtered_vehicle_topic_;
  std::string vehicle_frame_;
  onion_cloud_preprocessing::CropBounds bounds_;
  double tf_timeout_sec_ = 2.0;
  bool fail_if_tf_unavailable_ = true;
  int subscriber_queue_size_ = 10;
  int publisher_queue_size_ = 10;

  bool transform_cached_ = false;
  std::string cached_source_frame_;
  Eigen::Isometry3d vehicle_from_sensor_ =
      Eigen::Isometry3d::Identity();
  std::size_t cumulative_finite_points_ = 0;
  std::size_t cumulative_removed_points_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "onion_vehicle_crop_filter");
  try {
    VehicleCropFilterNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Vehicle crop filter failed: %s", error.what());
    return 2;
  }
  return 0;
}
