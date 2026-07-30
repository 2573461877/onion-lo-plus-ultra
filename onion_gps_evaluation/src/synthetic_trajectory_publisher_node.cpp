#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

void setYaw(double yaw, geometry_msgs::Quaternion* orientation) {
  orientation->x = 0.0;
  orientation->y = 0.0;
  orientation->z = std::sin(0.5 * yaw);
  orientation->w = std::cos(0.5 * yaw);
}

void trueLocalPose(double elapsed, double* x, double* y, double* z,
                   double* yaw) {
  *x = 1.6 * elapsed;
  *y = 3.0 * std::sin(0.18 * elapsed);
  *z = 0.15 * std::sin(0.08 * elapsed);
  *yaw = std::atan2(0.54 * std::cos(0.18 * elapsed), 1.6);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "synthetic_gps_trajectory_publisher");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  double duration_sec = 12.0;
  double rate_hz = 20.0;
  double event_time_sec = 5.0;
  std::string finalize_service =
      "/onion_segmented_registration_evaluation/finalize";
  private_node_handle.param("duration_sec", duration_sec, 12.0);
  private_node_handle.param("rate_hz", rate_hz, 20.0);
  private_node_handle.param("event_time_sec", event_time_sec, 5.0);
  private_node_handle.param<std::string>(
      "finalize_service", finalize_service,
      "/onion_segmented_registration_evaluation/finalize");
  if (duration_sec <= 0.0 || rate_hz <= 0.0) {
    ROS_FATAL("duration_sec and rate_hz must be positive");
    return 1;
  }

  ros::Publisher onion_publisher =
      node_handle.advertise<nav_msgs::Odometry>(
          "/onion_lo_plus_node/odometry", 100);
  ros::Publisher gps_odometry_publisher =
      node_handle.advertise<nav_msgs::Odometry>(
          "/gps/evaluation/odom_utm", 100);
  ros::Publisher gps_fix_publisher =
      node_handle.advertise<sensor_msgs::NavSatFix>(
          "/gps/evaluation/fix", 100);
  ros::Publisher gps_position_valid_publisher =
      node_handle.advertise<std_msgs::Bool>(
          "/gps/evaluation/position_valid", 10, true);
  ros::Publisher gps_heading_valid_publisher =
      node_handle.advertise<std_msgs::Bool>(
          "/gps/evaluation/heading_valid", 10, true);
  ros::Publisher initialpose_publisher =
      node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "/initialpose", 1, true);
  ros::Publisher status_publisher =
      node_handle.advertise<std_msgs::String>(
          "/onion_scancontext_relocalization/status", 5, true);

  constexpr double transform_yaw = 23.0 * kPi / 180.0;
  constexpr double translation_x = 500000.0;
  constexpr double translation_y = 4400000.0;
  constexpr double translation_z = 52.0;
  constexpr double base_latitude = 39.9;
  constexpr double base_longitude = 116.3;
  constexpr double earth_radius = 6378137.0;

  std_msgs::Bool valid;
  valid.data = true;
  gps_position_valid_publisher.publish(valid);
  gps_heading_valid_publisher.publish(valid);
  ros::Duration(1.0).sleep();

  const ros::Time start_stamp = ros::Time::now();
  ros::Rate rate(rate_hz);
  bool event_published = false;
  const int sample_count =
      static_cast<int>(std::floor(duration_sec * rate_hz)) + 1;
  for (int index = 0; index < sample_count && ros::ok(); ++index) {
    const double elapsed = static_cast<double>(index) / rate_hz;
    const ros::Time stamp =
        start_stamp + ros::Duration(elapsed);
    double true_x = 0.0;
    double true_y = 0.0;
    double true_z = 0.0;
    double true_yaw = 0.0;
    trueLocalPose(elapsed, &true_x, &true_y, &true_z, &true_yaw);
    const double cosine = std::cos(transform_yaw);
    const double sine = std::sin(transform_yaw);
    const double gps_x =
        cosine * true_x - sine * true_y + translation_x;
    const double gps_y =
        sine * true_x + cosine * true_y + translation_y;
    const double gps_z = true_z + translation_z;
    const double gps_yaw = true_yaw + transform_yaw;

    const double noise_x = 0.012 * std::sin(1.7 * elapsed);
    const double noise_y = 0.010 * std::cos(1.3 * elapsed);
    const double noise_z = 0.008 * std::sin(0.9 * elapsed);
    const double noise_yaw =
        0.15 * kPi / 180.0 * std::sin(0.8 * elapsed);
    double injected_x = 0.0;
    double injected_y = 0.0;
    double injected_yaw = 0.0;
    if (elapsed >= event_time_sec) {
      const double decay = std::exp(-(elapsed - event_time_sec));
      injected_x = 0.75 * decay;
      injected_y = -0.35 * decay;
      injected_yaw = 4.0 * kPi / 180.0 * decay;
    }

    nav_msgs::Odometry onion;
    onion.header.stamp = stamp;
    onion.header.frame_id = "odom";
    onion.child_frame_id = "vehicle_link";
    onion.pose.pose.position.x = true_x + noise_x + injected_x;
    onion.pose.pose.position.y = true_y + noise_y + injected_y;
    onion.pose.pose.position.z = true_z + noise_z;
    setYaw(true_yaw + noise_yaw + injected_yaw,
           &onion.pose.pose.orientation);

    nav_msgs::Odometry gps_odometry;
    gps_odometry.header.stamp = stamp;
    gps_odometry.header.frame_id = "utm";
    gps_odometry.child_frame_id = "gps_link";
    gps_odometry.pose.pose.position.x = gps_x;
    gps_odometry.pose.pose.position.y = gps_y;
    gps_odometry.pose.pose.position.z = gps_z;
    setYaw(gps_yaw, &gps_odometry.pose.pose.orientation);
    gps_odometry.pose.covariance[0] = 0.02 * 0.02;
    gps_odometry.pose.covariance[7] = 0.02 * 0.02;
    gps_odometry.pose.covariance[14] = 0.04 * 0.04;
    gps_odometry.pose.covariance[35] =
        std::pow(0.2 * kPi / 180.0, 2.0);

    const double east_offset = gps_x - translation_x;
    const double north_offset = gps_y - translation_y;
    sensor_msgs::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = "gps_link";
    fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    fix.latitude =
        base_latitude + north_offset / earth_radius * 180.0 / kPi;
    fix.longitude =
        base_longitude +
        east_offset /
            (earth_radius * std::cos(base_latitude * kPi / 180.0)) *
            180.0 / kPi;
    fix.altitude = gps_z;
    fix.position_covariance[0] = 0.02 * 0.02;
    fix.position_covariance[4] = 0.02 * 0.02;
    fix.position_covariance[8] = 0.04 * 0.04;
    fix.position_covariance_type =
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    gps_position_valid_publisher.publish(valid);
    gps_heading_valid_publisher.publish(valid);
    gps_fix_publisher.publish(fix);
    gps_odometry_publisher.publish(gps_odometry);
    if (!event_published && elapsed >= event_time_sec) {
      geometry_msgs::PoseWithCovarianceStamped initialpose;
      initialpose.header.stamp = stamp;
      initialpose.header.frame_id = "odom";
      initialpose.pose.pose = onion.pose.pose;
      initialpose_publisher.publish(initialpose);
      std_msgs::String status;
      status.data = "accepted entry=synthetic_reference";
      status_publisher.publish(status);
      event_published = true;
    }
    onion_publisher.publish(onion);
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Synthetic trajectory published: samples=%d duration=%.2f s",
           sample_count, duration_sec);
  if (!ros::service::waitForService(finalize_service,
                                    ros::Duration(10.0))) {
    ROS_FATAL_STREAM("Finalize service unavailable: "
                     << finalize_service);
    return 1;
  }
  std_srvs::Trigger finalize;
  if (!ros::service::call(finalize_service, finalize) ||
      !finalize.response.success) {
    ROS_FATAL_STREAM(
        "Synthetic finalization failed: "
        << finalize.response.message);
    return 1;
  }
  ROS_INFO_STREAM("Synthetic report: " << finalize.response.message);
  return 0;
}
