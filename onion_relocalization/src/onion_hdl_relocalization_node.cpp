#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

namespace onion_relocalization {

std::string NormalizeFrameId(const std::string& frame_id) {
  const auto first_character = frame_id.find_first_not_of('/');
  if (first_character == std::string::npos) return {};
  return frame_id.substr(first_character);
}

struct RefinedCandidate {
  std::size_t index = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double hdl_inlier_fraction = 0.0;
  double hdl_matching_error = std::numeric_limits<double>::max();
  double validation_inlier_fraction = 0.0;
  double validation_rmse = std::numeric_limits<double>::max();
  double refinement_translation = std::numeric_limits<double>::max();
  double refinement_rotation_deg = std::numeric_limits<double>::max();
  int supported_azimuth_sectors = 0;
  double duration_ms = 0.0;
  bool accepted = false;
};

class RelocalizationAdapter {
 public:
  RelocalizationAdapter()
      : nh_(), private_nh_("~"), last_attempt_(0.0) {
    private_nh_.param("map_path", map_path_, std::string());
    private_nh_.param("map_frame", map_frame_, std::string("odom"));
    private_nh_.param("lidar_topic", lidar_topic_,
                      std::string("/livox/lidar"));
    private_nh_.param("query_frame", query_frame_, std::string());
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
    private_nh_.param("enable_candidate_refinement",
                      enable_candidate_refinement_, true);
    private_nh_.param("refinement_map_path", refinement_map_path_,
                      map_path_);
    private_nh_.param("refinement_target_leaf_size",
                      refinement_target_leaf_size_, 0.20);
    private_nh_.param("refinement_query_leaf_size",
                      refinement_query_leaf_size_, 0.20);
    private_nh_.param("refinement_submap_xy_radius",
                      refinement_submap_xy_radius_, 80.0);
    private_nh_.param("refinement_submap_z_radius",
                      refinement_submap_z_radius_, 8.0);
    private_nh_.param("ndt_resolution", ndt_resolution_, 1.0);
    private_nh_.param("ndt_step_size", ndt_step_size_, 0.20);
    private_nh_.param("ndt_transformation_epsilon",
                      ndt_transformation_epsilon_, 0.01);
    private_nh_.param("ndt_max_iterations", ndt_max_iterations_, 40);
    private_nh_.param("icp_max_correspondence_distance",
                      icp_max_correspondence_distance_, 0.60);
    private_nh_.param("icp_transformation_epsilon",
                      icp_transformation_epsilon_, 1e-5);
    private_nh_.param("icp_euclidean_fitness_epsilon",
                      icp_euclidean_fitness_epsilon_, 1e-5);
    private_nh_.param("icp_max_iterations", icp_max_iterations_, 50);
    private_nh_.param("validation_max_correspondence_distance",
                      validation_max_correspondence_distance_, 0.30);
    private_nh_.param("validation_min_inlier_fraction",
                      validation_min_inlier_fraction_, 0.45);
    private_nh_.param("validation_max_rmse",
                      validation_max_rmse_, 0.18);
    private_nh_.param("validation_azimuth_sectors",
                      validation_azimuth_sectors_, 12);
    private_nh_.param("validation_min_azimuth_sectors",
                      validation_min_azimuth_sectors_, 6);
    private_nh_.param("validation_min_sector_inliers",
                      validation_min_sector_inliers_, 5);
    private_nh_.param("max_refinement_translation",
                      max_refinement_translation_, 10.0);
    private_nh_.param("max_refinement_rotation_deg",
                      max_refinement_rotation_deg_, 25.0);
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
    if (enable_candidate_refinement_) {
      LoadRefinementMap();
    }

    lidar_subscriber_ =
        nh_.subscribe(lidar_topic_, 1,
                      &RelocalizationAdapter::PointCloudCallback, this,
                      ros::TransportHints().tcpNoDelay());
    relocalization_pending_ = auto_relocalize_;
    query_frame_ = NormalizeFrameId(query_frame_);

    ROS_INFO(
        "Onion global relocalization adapter ready: map='%s', "
        "LiDAR='%s', query/output child='%s', auto=%s",
        map_path_.c_str(), lidar_topic_.c_str(),
        query_frame_.empty() ? "<cloud frame>" : query_frame_.c_str(),
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
    if (enable_candidate_refinement_ && refinement_map_path_.empty()) {
      throw std::runtime_error(
          "~refinement_map_path must not be empty when refinement is enabled");
    }
    if (refinement_target_leaf_size_ <= 0.0 ||
        refinement_query_leaf_size_ <= 0.0 ||
        refinement_submap_xy_radius_ <= 0.0 ||
        refinement_submap_z_radius_ <= 0.0 ||
        ndt_resolution_ <= 0.0 || ndt_step_size_ <= 0.0 ||
        ndt_transformation_epsilon_ <= 0.0 ||
        ndt_max_iterations_ <= 0 ||
        icp_max_correspondence_distance_ <= 0.0 ||
        icp_transformation_epsilon_ <= 0.0 ||
        icp_euclidean_fitness_epsilon_ <= 0.0 ||
        icp_max_iterations_ <= 0 ||
        validation_max_correspondence_distance_ <= 0.0 ||
        validation_max_rmse_ <= 0.0 ||
        max_refinement_translation_ <= 0.0 ||
        max_refinement_rotation_deg_ <= 0.0) {
      throw std::runtime_error(
          "candidate-refinement distances, iterations, and tolerances "
          "must be positive");
    }
    if (validation_min_inlier_fraction_ < 0.0 ||
        validation_min_inlier_fraction_ > 1.0) {
      throw std::runtime_error(
          "~validation_min_inlier_fraction must be in [0, 1]");
    }
    if (validation_azimuth_sectors_ <= 0 ||
        validation_min_azimuth_sectors_ <= 0 ||
        validation_min_azimuth_sectors_ > validation_azimuth_sectors_ ||
        validation_min_sector_inliers_ <= 0) {
      throw std::runtime_error(
          "azimuth-sector validation parameters are inconsistent");
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

  void LoadRefinementMap() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(refinement_map_path_, *raw_map) != 0) {
      throw std::runtime_error(
          "failed to load refinement PCD: " + refinement_map_path_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr finite_map(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> retained_indices;
    pcl::removeNaNFromPointCloud(
        *raw_map, *finite_map, retained_indices);
    if (finite_map->empty()) {
      throw std::runtime_error(
          "refinement PCD contains no finite XYZ points: " +
          refinement_map_path_);
    }

    refinement_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(
        refinement_target_leaf_size_,
        refinement_target_leaf_size_,
        refinement_target_leaf_size_);
    voxel_grid.setInputCloud(finite_map);
    voxel_grid.filter(*refinement_map_);
    if (refinement_map_->empty()) {
      throw std::runtime_error(
          "refinement PCD became empty after voxel filtering");
    }

    ROS_INFO(
        "Loaded refinement map '%s': %zu finite -> %zu voxel points "
        "(leaf=%.3f m)",
        refinement_map_path_.c_str(), finite_map->size(),
        refinement_map_->size(), refinement_target_leaf_size_);
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
    const std::string cloud_frame =
        NormalizeFrameId(cloud->header.frame_id);
    if (cloud_frame.empty() ||
        (!query_frame_.empty() && cloud_frame != query_frame_)) {
      HandleFailure(
          "HDL query cloud frame '" + cloud->header.frame_id +
          "' does not match required pose frame '" + query_frame_ + "'");
      return;
    }
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
    const ros::WallTime query_started = ros::WallTime::now();
    if (!query_client_.call(service)) {
      HandleFailure("HDL query service returned no solution");
      return;
    }

    const std::size_t candidate_count =
        std::min(service.response.poses.size(),
                 std::min(service.response.inlier_fractions.size(),
                          service.response.errors.size()));
    const double query_duration_ms =
        (ros::WallTime::now() - query_started).toSec() * 1000.0;
    ROS_INFO("HDL query completed in %.3f ms with %zu candidates",
             query_duration_ms, candidate_count);
    if (candidate_count == 0) {
      HandleFailure("HDL query returned empty candidate arrays");
      return;
    }

    for (std::size_t i = 0; i < candidate_count; ++i) {
      const geometry_msgs::Pose& candidate_pose =
          service.response.poses[i];
      ROS_INFO(
          "HDL candidate[%zu]: xyz=(%.3f, %.3f, %.3f), "
          "inlier_fraction=%.4f, error=%.4f",
          i, candidate_pose.position.x, candidate_pose.position.y,
          candidate_pose.position.z,
          service.response.inlier_fractions[i],
          service.response.errors[i]);
    }

    if (enable_candidate_refinement_) {
      geometry_msgs::Pose refined_pose;
      double refined_inlier_fraction = 0.0;
      double refined_rmse = std::numeric_limits<double>::max();
      if (!RefineCandidates(
              *cloud, service.response.poses,
              service.response.inlier_fractions,
              service.response.errors, candidate_count,
              refined_pose, refined_inlier_fraction, refined_rmse)) {
        HandleFailure(
            "all HDL Top-K candidates failed dense-map refinement "
            "or geometry validation");
        return;
      }
      PublishInitialPose(
          refined_pose, cloud->header.stamp,
          refined_inlier_fraction, refined_rmse);
      relocalization_pending_ = false;
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

  bool RefineCandidates(
      const sensor_msgs::PointCloud2& cloud_message,
      const std::vector<geometry_msgs::Pose>& poses,
      const std::vector<double>& hdl_inlier_fractions,
      const std::vector<double>& hdl_errors,
      std::size_t candidate_count,
      geometry_msgs::Pose& best_pose,
      double& best_inlier_fraction,
      double& best_rmse) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_query(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_message, *raw_query);
    pcl::PointCloud<pcl::PointXYZ>::Ptr finite_query(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> retained_indices;
    pcl::removeNaNFromPointCloud(
        *raw_query, *finite_query, retained_indices);
    if (finite_query->empty()) {
      ROS_WARN("Dense-map refinement received no finite query points");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr query(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(
        refinement_query_leaf_size_,
        refinement_query_leaf_size_,
        refinement_query_leaf_size_);
    voxel_grid.setInputCloud(finite_query);
    voxel_grid.filter(*query);
    if (query->size() < 20) {
      ROS_WARN(
          "Dense-map refinement query has only %zu voxel points",
          query->size());
      return false;
    }

    ROS_INFO(
        "Refining %zu HDL candidates with %zu query voxel points",
        candidate_count, query->size());
    std::vector<RefinedCandidate> refined_candidates;
    refined_candidates.reserve(candidate_count);
    for (std::size_t i = 0; i < candidate_count; ++i) {
      if (!PoseIsFinite(poses[i]) ||
          !std::isfinite(hdl_inlier_fractions[i]) ||
          !std::isfinite(hdl_errors[i])) {
        ROS_WARN("Skipping non-finite HDL candidate[%zu]", i);
        continue;
      }

      RefinedCandidate candidate;
      candidate.index = i;
      candidate.hdl_inlier_fraction = hdl_inlier_fractions[i];
      candidate.hdl_matching_error = hdl_errors[i];
      RefineCandidate(*query, poses[i], candidate);
      refined_candidates.push_back(candidate);
    }

    const RefinedCandidate* best = nullptr;
    for (const RefinedCandidate& candidate : refined_candidates) {
      if (!candidate.accepted) continue;
      if (best == nullptr ||
          CandidateScore(candidate) < CandidateScore(*best) ||
          (CandidateScore(candidate) == CandidateScore(*best) &&
           candidate.validation_inlier_fraction >
               best->validation_inlier_fraction)) {
        best = &candidate;
      }
    }
    if (best == nullptr) return false;

    best_pose = MatrixToPose(best->pose);
    best_inlier_fraction = best->validation_inlier_fraction;
    best_rmse = best->validation_rmse;
    ROS_INFO(
        "Selected refined candidate[%zu]: xyz=(%.3f, %.3f, %.3f), "
        "validation_inlier=%.4f, rmse=%.4f m, sectors=%d/%d, "
        "score=%.4f, correction=%.3f m/%.2f deg, refine_time=%.1f ms",
        best->index, best_pose.position.x, best_pose.position.y,
        best_pose.position.z, best->validation_inlier_fraction,
        best->validation_rmse, best->supported_azimuth_sectors,
        validation_azimuth_sectors_, CandidateScore(*best),
        best->refinement_translation, best->refinement_rotation_deg,
        best->duration_ms);
    return true;
  }

  double CandidateScore(
      const RefinedCandidate& candidate) const {
    return candidate.validation_rmse /
           std::max(candidate.validation_inlier_fraction, 1e-6);
  }

  void RefineCandidate(
      const pcl::PointCloud<pcl::PointXYZ>& query,
      const geometry_msgs::Pose& hdl_pose,
      RefinedCandidate& result) {
    const ros::WallTime started = ros::WallTime::now();
    const Eigen::Matrix4f initial_guess = PoseToMatrix(hdl_pose);

    pcl::PointCloud<pcl::PointXYZ>::Ptr submap(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> crop_box;
    const Eigen::Vector3f center = initial_guess.block<3, 1>(0, 3);
    crop_box.setMin(Eigen::Vector4f(
        center.x() - refinement_submap_xy_radius_,
        center.y() - refinement_submap_xy_radius_,
        center.z() - refinement_submap_z_radius_, 1.0f));
    crop_box.setMax(Eigen::Vector4f(
        center.x() + refinement_submap_xy_radius_,
        center.y() + refinement_submap_xy_radius_,
        center.z() + refinement_submap_z_radius_, 1.0f));
    crop_box.setInputCloud(refinement_map_);
    crop_box.filter(*submap);
    if (submap->size() < 100) {
      result.duration_ms =
          (ros::WallTime::now() - started).toSec() * 1000.0;
      ROS_WARN(
          "Refined candidate[%zu] rejected: local submap has only "
          "%zu points",
          result.index, submap->size());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr query_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(query));
    pcl::PointCloud<pcl::PointXYZ> ndt_aligned;
    pcl::NormalDistributionsTransform<
        pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(query_ptr);
    ndt.setInputTarget(submap);
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt.setMaximumIterations(ndt_max_iterations_);
    ndt.align(ndt_aligned, initial_guess);
    if (!ndt.hasConverged()) {
      result.duration_ms =
          (ros::WallTime::now() - started).toSec() * 1000.0;
      ROS_WARN(
          "Refined candidate[%zu] rejected: coarse NDT did not converge "
          "(submap=%zu)",
          result.index, submap->size());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> icp_aligned;
    pcl::IterativeClosestPoint<
        pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(query_ptr);
    icp.setInputTarget(submap);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setMaxCorrespondenceDistance(
        icp_max_correspondence_distance_);
    icp.setTransformationEpsilon(icp_transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(
        icp_euclidean_fitness_epsilon_);
    icp.align(icp_aligned, ndt.getFinalTransformation());
    if (!icp.hasConverged()) {
      result.duration_ms =
          (ros::WallTime::now() - started).toSec() * 1000.0;
      ROS_WARN(
          "Refined candidate[%zu] rejected: fine ICP did not converge",
          result.index);
      return;
    }

    result.pose = icp.getFinalTransformation();
    const Eigen::Matrix4f correction =
        initial_guess.inverse() * result.pose;
    result.refinement_translation =
        correction.block<3, 1>(0, 3).norm();
    const Eigen::Matrix3f correction_rotation =
        correction.block<3, 3>(0, 0);
    result.refinement_rotation_deg =
        Eigen::AngleAxisf(correction_rotation).angle() *
        180.0 / 3.14159265358979323846;

    ValidateRefinedPose(
        query, *submap, result.pose,
        result.validation_inlier_fraction,
        result.validation_rmse,
        result.supported_azimuth_sectors);
    result.duration_ms =
        (ros::WallTime::now() - started).toSec() * 1000.0;
    result.accepted =
        std::isfinite(result.validation_inlier_fraction) &&
        std::isfinite(result.validation_rmse) &&
        result.validation_inlier_fraction >=
            validation_min_inlier_fraction_ &&
        result.validation_rmse <= validation_max_rmse_ &&
        result.supported_azimuth_sectors >=
            validation_min_azimuth_sectors_ &&
        result.refinement_translation <=
            max_refinement_translation_ &&
        result.refinement_rotation_deg <=
            max_refinement_rotation_deg_;

    ROS_INFO(
        "Refined candidate[%zu]: accepted=%s, xyz=(%.3f, %.3f, %.3f), "
        "HDL(inlier=%.4f,error=%.4f), dense(inlier=%.4f,rmse=%.4f m,"
        "sectors=%d/%d), correction=%.3f m/%.2f deg, "
        "submap=%zu, time=%.1f ms",
        result.index, result.accepted ? "true" : "false",
        result.pose(0, 3), result.pose(1, 3), result.pose(2, 3),
        result.hdl_inlier_fraction, result.hdl_matching_error,
        result.validation_inlier_fraction, result.validation_rmse,
        result.supported_azimuth_sectors,
        validation_azimuth_sectors_, result.refinement_translation,
        result.refinement_rotation_deg, submap->size(),
        result.duration_ms);
  }

  void ValidateRefinedPose(
      const pcl::PointCloud<pcl::PointXYZ>& query,
      const pcl::PointCloud<pcl::PointXYZ>& target,
      const Eigen::Matrix4f& pose,
      double& inlier_fraction,
      double& rmse,
      int& supported_azimuth_sectors) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(target));
    pcl::KdTreeFLANN<pcl::PointXYZ> target_tree;
    target_tree.setInputCloud(target_ptr);

    pcl::PointCloud<pcl::PointXYZ> transformed_query;
    pcl::transformPointCloud(query, transformed_query, pose);
    const double max_squared_distance =
        validation_max_correspondence_distance_ *
        validation_max_correspondence_distance_;
    std::vector<int> sector_inliers(
        validation_azimuth_sectors_, 0);
    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_squared_distances(1);
    std::size_t inlier_count = 0;
    double squared_error_sum = 0.0;
    const double two_pi = 2.0 * 3.14159265358979323846;

    for (std::size_t i = 0; i < transformed_query.size(); ++i) {
      if (target_tree.nearestKSearch(
              transformed_query[i], 1, nearest_indices,
              nearest_squared_distances) <= 0 ||
          nearest_squared_distances[0] > max_squared_distance) {
        continue;
      }
      ++inlier_count;
      squared_error_sum += nearest_squared_distances[0];
      double angle = std::atan2(query[i].y, query[i].x);
      if (angle < 0.0) angle += two_pi;
      int sector = static_cast<int>(
          angle / two_pi * validation_azimuth_sectors_);
      sector = std::max(
          0, std::min(validation_azimuth_sectors_ - 1, sector));
      ++sector_inliers[sector];
    }

    inlier_fraction = query.empty()
                          ? 0.0
                          : static_cast<double>(inlier_count) /
                                static_cast<double>(query.size());
    rmse = inlier_count == 0
               ? std::numeric_limits<double>::max()
               : std::sqrt(
                     squared_error_sum /
                     static_cast<double>(inlier_count));
    supported_azimuth_sectors = static_cast<int>(std::count_if(
        sector_inliers.begin(), sector_inliers.end(),
        [this](int count) {
          return count >= validation_min_sector_inliers_;
        }));
  }

  Eigen::Matrix4f PoseToMatrix(
      const geometry_msgs::Pose& pose) const {
    Eigen::Quaternionf quaternion(
        pose.orientation.w, pose.orientation.x,
        pose.orientation.y, pose.orientation.z);
    quaternion.normalize();
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    matrix(0, 3) = pose.position.x;
    matrix(1, 3) = pose.position.y;
    matrix(2, 3) = pose.position.z;
    return matrix;
  }

  geometry_msgs::Pose MatrixToPose(
      const Eigen::Matrix4f& matrix) const {
    geometry_msgs::Pose pose;
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    Eigen::Quaternionf quaternion(
        matrix.block<3, 3>(0, 0));
    quaternion.normalize();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return pose;
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
        "Published HDL /initialpose for child '%s': "
        "xyz=(%.3f, %.3f, %.3f), "
        "inlier_fraction=%.4f, error=%.4f",
        query_frame_.empty() ? "<cloud frame>" : query_frame_.c_str(),
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
  std::string query_frame_;
  std::string set_map_service_;
  std::string query_service_;
  std::string initialpose_topic_;
  std::string refinement_map_path_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr refinement_map_;
  bool auto_relocalize_ = true;
  bool enable_candidate_refinement_ = true;
  bool relocalization_pending_ = false;
  int max_num_candidates_ = 3;
  int max_query_attempts_ = 3;
  int attempts_ = 0;
  double retry_delay_sec_ = 1.0;
  double min_inlier_fraction_ = 0.25;
  double max_matching_error_ = -1.0;
  double service_wait_timeout_sec_ = 30.0;
  double refinement_target_leaf_size_ = 0.20;
  double refinement_query_leaf_size_ = 0.20;
  double refinement_submap_xy_radius_ = 80.0;
  double refinement_submap_z_radius_ = 8.0;
  double ndt_resolution_ = 1.0;
  double ndt_step_size_ = 0.20;
  double ndt_transformation_epsilon_ = 0.01;
  int ndt_max_iterations_ = 40;
  double icp_max_correspondence_distance_ = 0.60;
  double icp_transformation_epsilon_ = 1e-5;
  double icp_euclidean_fitness_epsilon_ = 1e-5;
  int icp_max_iterations_ = 50;
  double validation_max_correspondence_distance_ = 0.30;
  double validation_min_inlier_fraction_ = 0.45;
  double validation_max_rmse_ = 0.18;
  int validation_azimuth_sectors_ = 12;
  int validation_min_azimuth_sectors_ = 6;
  int validation_min_sector_inliers_ = 5;
  double max_refinement_translation_ = 10.0;
  double max_refinement_rotation_deg_ = 25.0;
  double position_stddev_ = 0.5;
  double orientation_stddev_ = 0.35;
  ros::WallTime last_attempt_;
};

}  // namespace onion_relocalization

int main(int argc, char** argv) {
  ros::init(argc, argv, "onion_hdl_relocalization");
  try {
    onion_relocalization::RelocalizationAdapter adapter;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("Failed to initialize global relocalization adapter: %s",
              exception.what());
    return 1;
  }
  return 0;
}
