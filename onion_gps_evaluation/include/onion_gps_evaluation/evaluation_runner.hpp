#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "onion_gps_evaluation/trajectory_alignment.hpp"

namespace onion_gps_evaluation {

enum class EvaluationWorkflow {
  kTrajectoryRegistration,
  kLocalizationEvaluation,
  kSegmentedRegistrationEvaluation,
};

class EvaluationRunner {
 public:
  EvaluationRunner(ros::NodeHandle node_handle,
                   ros::NodeHandle private_node_handle,
                   EvaluationWorkflow workflow);
  ~EvaluationRunner();

  struct RelocalizationEvent {
    double stamp = 0.0;
    PoseSample pose;
    std::string frame_id;
    std::string source;
  };

  struct StatusEvent {
    double stamp = 0.0;
    std::string status;
  };

 private:
  enum class ValidityState { kNotReceived, kInvalid, kValid };

  struct NavSatReference {
    double latitude_deg = kNaN;
    double longitude_deg = kNaN;
    double altitude_m = kNaN;
  };

  void loadParameters();
  void validateParameters() const;
  void setUpRosInterfaces();
  void loadAlignment();

  void onionCallback(const nav_msgs::Odometry::ConstPtr& message);
  void gpsOdometryCallback(const nav_msgs::Odometry::ConstPtr& message);
  void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& message);
  void gpsPositionValidityCallback(const std_msgs::Bool::ConstPtr& message);
  void gpsHeadingValidityCallback(const std_msgs::Bool::ConstPtr& message);
  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);
  void relocalizationStatusCallback(
      const std_msgs::String::ConstPtr& message);
  bool finalizeService(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);
  void shutdownTimerCallback(const ros::WallTimerEvent&);

  bool gpsPositionAccepted();
  PoseSample poseSampleFromOdometry(const nav_msgs::Odometry& message,
                                    const std::string& source) const;
  std::vector<PoseSample> referenceSamples() const;
  void attachNavSatCoordinates(std::vector<PoseSample>* references) const;

  bool finalize(std::string* result_message);
  void publishPaths(const std::vector<MatchedPose>& matches,
                    const PlanarTransform& transform);

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  EvaluationWorkflow workflow_;

  std::string workflow_name_;
  std::string gps_position_source_;
  std::string onion_odom_topic_;
  std::string gps_odom_topic_;
  std::string gps_fix_topic_;
  std::string gps_position_valid_topic_;
  std::string gps_heading_valid_topic_;
  std::string initialpose_topic_;
  std::string relocalization_status_topic_;
  std::string output_directory_;
  std::string alignment_input_path_;
  std::string alignment_output_path_;

  bool require_gps_position_valid_ = true;
  bool require_gps_heading_valid_for_yaw_ = true;
  bool shutdown_after_finalize_ = false;
  double max_time_difference_sec_ = 0.15;
  double max_interpolation_gap_sec_ = 0.30;
  int minimum_matched_poses_ = 30;
  int minimum_registration_poses_ = 20;
  double minimum_registration_baseline_m_ = 5.0;
  double registration_fraction_ = 0.30;
  double registration_duration_sec_ = 0.0;
  double robust_outlier_threshold_m_ = 0.75;
  int robust_ransac_iterations_ = 300;
  int random_seed_ = 7;
  std::vector<double> accuracy_thresholds_m_;
  double rpe_delta_sec_ = 1.0;
  double convergence_threshold_m_ = 0.20;
  int convergence_hold_samples_ = 10;
  double relocalization_timeout_sec_ = 10.0;

  std::optional<NavSatReference> navsat_reference_;
  PlanarTransform loaded_transform_;
  bool has_loaded_transform_ = false;

  mutable std::mutex mutex_;
  bool finalized_ = false;
  bool finalizing_ = false;
  ValidityState latest_gps_position_validity_ =
      ValidityState::kNotReceived;
  ValidityState latest_gps_heading_validity_ =
      ValidityState::kNotReceived;
  std::size_t dropped_invalid_gps_count_ = 0;
  std::size_t dropped_before_position_validity_count_ = 0;
  std::size_t suppressed_invalid_gps_yaw_count_ = 0;
  std::size_t suppressed_before_heading_validity_count_ = 0;
  std::vector<PoseSample> onion_samples_;
  std::vector<PoseSample> gps_odometry_samples_;
  std::vector<PoseSample> gps_fix_samples_;
  std::vector<RelocalizationEvent> relocalization_events_;
  std::vector<StatusEvent> status_events_;

  ros::Subscriber onion_subscriber_;
  ros::Subscriber gps_odometry_subscriber_;
  ros::Subscriber gps_fix_subscriber_;
  ros::Subscriber gps_position_validity_subscriber_;
  ros::Subscriber gps_heading_validity_subscriber_;
  ros::Subscriber initialpose_subscriber_;
  ros::Subscriber relocalization_status_subscriber_;
  ros::Publisher summary_publisher_;
  ros::Publisher gps_path_publisher_;
  ros::Publisher aligned_onion_path_publisher_;
  ros::ServiceServer finalize_service_;
  ros::WallTimer shutdown_timer_;
};

}  // namespace onion_gps_evaluation
