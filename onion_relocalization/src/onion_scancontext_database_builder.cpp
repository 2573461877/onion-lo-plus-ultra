#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <onion_cloud_preprocessing/pointcloud_crop.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "Scancontext.h"
#include "onion_relocalization/descriptor_io.hpp"
#include "onion_relocalization/pointcloud_deskew.hpp"

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;
using onion_relocalization::PointTimeConfig;
using onion_relocalization::TimedPose;

std::string normalizeFrameId(const std::string& frame_id) {
  const auto first_character = frame_id.find_first_not_of('/');
  if (first_character == std::string::npos) return {};
  return frame_id.substr(first_character);
}

Eigen::Isometry3d transformToEigen(
    const geometry_msgs::TransformStamped& transform) {
  const auto& rotation = transform.transform.rotation;
  Eigen::Quaterniond quaternion(
      rotation.w, rotation.x, rotation.y, rotation.z);
  if (!quaternion.coeffs().allFinite() ||
      quaternion.norm() < 1.0e-9) {
    throw std::runtime_error("preprocessing TF has an invalid quaternion");
  }
  quaternion.normalize();
  const auto& translation = transform.transform.translation;
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.linear() = quaternion.toRotationMatrix();
  output.translation() =
      Eigen::Vector3d(translation.x, translation.y, translation.z);
  if (!output.matrix().allFinite()) {
    throw std::runtime_error("preprocessing TF is not finite");
  }
  return output;
}

struct PoseRecord {
  double stamp_sec = 0.0;
  double elapsed_sec = 0.0;
  Eigen::Isometry3d map_from_sensor = Eigen::Isometry3d::Identity();
};

struct Frame {
  PoseRecord pose;
  CloudPtr cloud{new Cloud};
};

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> output;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {
    output.push_back(value);
  }
  return output;
}

std::vector<PoseRecord> loadReferencePoses(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open reference pose CSV: " + path);
  }
  std::string line;
  std::getline(input, line);
  std::vector<PoseRecord> output;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto values = splitCsv(line);
    if (values.size() < 18) {
      throw std::runtime_error("malformed reference pose row: " + line);
    }
    PoseRecord pose;
    pose.stamp_sec = std::stod(values[2]);
    pose.elapsed_sec = std::stod(values[3]);
    pose.map_from_sensor.translation() =
        Eigen::Vector3d(std::stod(values[11]),
                        std::stod(values[12]),
                        std::stod(values[13]));
    Eigen::Quaterniond quaternion(std::stod(values[17]),
                                 std::stod(values[14]),
                                 std::stod(values[15]),
                                 std::stod(values[16]));
    quaternion.normalize();
    pose.map_from_sensor.linear() = quaternion.toRotationMatrix();
    output.push_back(std::move(pose));
  }
  if (output.empty()) {
    throw std::runtime_error("reference pose CSV contains no records");
  }
  return output;
}

std::vector<TimedPose> toTimedPoses(
    const std::vector<PoseRecord>& poses) {
  std::vector<TimedPose> output;
  output.reserve(poses.size());
  for (const auto& pose : poses) {
    TimedPose timed_pose;
    timed_pose.stamp_sec = pose.stamp_sec;
    timed_pose.pose = pose.map_from_sensor;
    output.push_back(std::move(timed_pose));
  }
  return output;
}

bool interpolateReferencePose(const std::vector<PoseRecord>& poses,
                              double stamp_sec,
                              double maximum_gap_sec,
                              PoseRecord* pose) {
  const auto upper =
      std::lower_bound(poses.begin(), poses.end(), stamp_sec,
                       [](const PoseRecord& sample, double value) {
                         return sample.stamp_sec < value;
                       });
  if (upper != poses.end() &&
      std::abs(upper->stamp_sec - stamp_sec) <= 1.0e-6) {
    *pose = *upper;
    return true;
  }
  if (upper == poses.begin() || upper == poses.end()) {
    return false;
  }
  const auto lower = std::prev(upper);
  const double interval = upper->stamp_sec - lower->stamp_sec;
  if (interval <= 0.0 || interval > maximum_gap_sec) {
    return false;
  }
  const double alpha =
      std::clamp((stamp_sec - lower->stamp_sec) / interval, 0.0, 1.0);
  Eigen::Quaterniond lower_rotation(lower->map_from_sensor.linear());
  Eigen::Quaterniond upper_rotation(upper->map_from_sensor.linear());
  lower_rotation.normalize();
  upper_rotation.normalize();
  pose->stamp_sec = stamp_sec;
  pose->elapsed_sec =
      (1.0 - alpha) * lower->elapsed_sec + alpha * upper->elapsed_sec;
  pose->map_from_sensor.translation() =
      (1.0 - alpha) * lower->map_from_sensor.translation() +
      alpha * upper->map_from_sensor.translation();
  pose->map_from_sensor.linear() =
      lower_rotation.slerp(alpha, upper_rotation).normalized()
          .toRotationMatrix();
  return true;
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

CloudPtr filterAndDownsample(const CloudPtr& input, float leaf_size) {
  CloudPtr filtered(new Cloud);
  filtered->reserve(input->size());
  for (const auto& point : input->points) {
    const float radius_sq = point.x * point.x + point.y * point.y;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || radius_sq < 4.0F ||
        radius_sq > 6400.0F || point.z < -4.0F || point.z > 12.0F) {
      continue;
    }
    filtered->push_back(point);
  }
  CloudPtr output(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(filtered);
  voxel.filter(*output);
  return output;
}

std::vector<Frame> loadFrames(
    const std::string& bag_path,
    const std::string& topic,
    const std::vector<PoseRecord>& poses,
    const PointTimeConfig& point_time_config,
    double minimum_deskewed_fraction,
    const std::string& preprocess_to_frame,
    const onion_cloud_preprocessing::CropBounds& crop_bounds,
    double tf_timeout_sec, tf2_ros::Buffer* transform_buffer) {
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery({topic}));
  std::vector<Frame> output;
  output.reserve(poses.size());
  const auto timed_poses = toTimedPoses(poses);
  std::size_t rejected_pose = 0;
  std::size_t rejected_deskew = 0;
  std::size_t cumulative_finite = 0;
  std::size_t cumulative_removed = 0;
  std::string cached_source_frame;
  Eigen::Isometry3d preprocess_from_source =
      Eigen::Isometry3d::Identity();
  for (const auto& message : view) {
    const auto cloud_message =
        message.instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_message) {
      continue;
    }
    const std::string source_frame =
        normalizeFrameId(cloud_message->header.frame_id);
    if (source_frame.empty()) {
      throw std::runtime_error(
          "database PointCloud2 has an empty frame_id");
    }
    if (cached_source_frame != source_frame) {
      if (source_frame == preprocess_to_frame) {
        preprocess_from_source = Eigen::Isometry3d::Identity();
      } else {
        try {
          preprocess_from_source = transformToEigen(
              transform_buffer->lookupTransform(
                  preprocess_to_frame, source_frame, ros::Time(0),
                  ros::Duration(tf_timeout_sec)));
        } catch (const tf2::TransformException& error) {
          throw std::runtime_error(
              "cannot preprocess database cloud from '" + source_frame +
              "' to '" + preprocess_to_frame + "': " + error.what());
        }
      }
      cached_source_frame = source_frame;
      ROS_INFO("database preprocessing cached TF %s <- %s",
               preprocess_to_frame.c_str(), source_frame.c_str());
    }
    auto cropped = onion_cloud_preprocessing::CropAndTransform(
        *cloud_message, preprocess_from_source, crop_bounds, false, true);
    cropped.vehicle_cloud.header.frame_id = preprocess_to_frame;
    cumulative_finite += cropped.finite_points;
    cumulative_removed += cropped.removed_vehicle_points;
    const sensor_msgs::PointCloud2& processed_message =
        cropped.vehicle_cloud;

    const double stamp = cloud_message->header.stamp.toSec();
    Frame frame;
    if (!interpolateReferencePose(
            poses, stamp, point_time_config.maximum_pose_gap_sec,
            &frame.pose)) {
      ++rejected_pose;
      continue;
    }
    CloudPtr raw(new Cloud);
    if (point_time_config.enabled) {
      const auto deskewed =
          onion_relocalization::deskewPointCloud(
              processed_message, point_time_config, timed_poses);
      const std::size_t total_points =
          deskewed.valid_points + deskewed.invalid_points;
      const double valid_fraction =
          total_points > 0
              ? static_cast<double>(deskewed.valid_points) /
                    static_cast<double>(total_points)
              : 0.0;
      if (!deskewed.ok() ||
          valid_fraction < minimum_deskewed_fraction) {
        ++rejected_deskew;
        ROS_WARN_STREAM_THROTTLE(
            1.0, "database builder rejected an un-deskewable frame: "
                     << (deskewed.error.empty()
                             ? "valid point fraction=" +
                                   std::to_string(valid_fraction)
                             : deskewed.error));
        continue;
      }
      raw = deskewed.cloud;
    } else {
      pcl::fromROSMsg(processed_message, *raw);
    }
    frame.cloud = filterAndDownsample(raw, 0.20F);
    output.push_back(std::move(frame));
  }
  bag.close();
  ROS_INFO(
      "database frame preparation: accepted=%zu rejected_pose=%zu "
      "rejected_deskew=%zu point_deskew=%s vehicle_crop=%.2f%% "
      "pose/cloud_frame=%s",
      output.size(), rejected_pose, rejected_deskew,
      point_time_config.enabled ? "enabled" : "disabled",
      cumulative_finite == 0
          ? 0.0
          : 100.0 * cumulative_removed /
                static_cast<double>(cumulative_finite),
      preprocess_to_frame.c_str());
  if (output.size() < 20) {
    throw std::runtime_error(
        "too few time-aligned point-cloud frames: " +
        std::to_string(output.size()));
  }
  return output;
}

std::size_t nearestFrame(const std::vector<Frame>& frames,
                         double elapsed_sec) {
  const auto upper =
      std::lower_bound(frames.begin(), frames.end(), elapsed_sec,
                       [](const Frame& frame, double value) {
                         return frame.pose.elapsed_sec < value;
                       });
  if (upper == frames.begin()) {
    return 0;
  }
  if (upper == frames.end()) {
    return frames.size() - 1;
  }
  const std::size_t upper_index =
      static_cast<std::size_t>(std::distance(frames.begin(), upper));
  const std::size_t lower_index = upper_index - 1;
  return std::abs(frames[upper_index].pose.elapsed_sec - elapsed_sec) <
                 std::abs(frames[lower_index].pose.elapsed_sec -
                          elapsed_sec)
             ? upper_index
             : lower_index;
}

CloudPtr aggregate(const std::vector<Frame>& frames,
                   std::size_t anchor_index,
                   double half_window_sec,
                   float leaf_size) {
  const auto& anchor = frames.at(anchor_index);
  const double begin = anchor.pose.elapsed_sec - half_window_sec;
  const double end = anchor.pose.elapsed_sec + half_window_sec;
  const Eigen::Isometry3d anchor_from_map =
      anchor.pose.map_from_sensor.inverse();
  CloudPtr accumulated(new Cloud);
  for (const auto& frame : frames) {
    if (frame.pose.elapsed_sec < begin) {
      continue;
    }
    if (frame.pose.elapsed_sec > end) {
      break;
    }
    Cloud transformed;
    const Eigen::Matrix4f anchor_from_frame =
        (anchor_from_map * frame.pose.map_from_sensor)
            .matrix()
            .cast<float>();
    pcl::transformPointCloud(*frame.cloud, transformed,
                             anchor_from_frame);
    *accumulated += transformed;
  }
  return filterAndDownsample(accumulated, leaf_size);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "onion_scancontext_database_builder",
            ros::init_options::AnonymousName);
  if (argc < 5 || argc > 7) {
    std::cerr
        << "Usage: " << argv[0]
        << " BAG POINTCLOUD_TOPIC REFERENCE_POSES_CSV OUTPUT_DIRECTORY "
           "[KEYFRAME_INTERVAL_SEC=2.0] [HALF_WINDOW_SEC=1.0]\n";
    return 1;
  }

  try {
    ros::NodeHandle private_node("~");
    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener(transform_buffer);
    PointTimeConfig point_time_config;
    private_node.param("enable_point_deskew",
                       point_time_config.enabled, true);
    private_node.param("require_point_deskew",
                       point_time_config.required, true);
    private_node.param<std::string>(
        "point_time_field", point_time_config.primary_field, "t_sec");
    private_node.param("point_time_scale",
                       point_time_config.primary_scale, 1.0);
    private_node.param<std::string>(
        "point_time_secondary_field",
        point_time_config.secondary_field, "t_usec");
    private_node.param("point_time_secondary_scale",
                       point_time_config.secondary_scale, 1.0e-6);
    private_node.param("point_time_is_offset",
                       point_time_config.time_is_offset, false);
    private_node.param("maximum_pose_interpolation_gap_sec",
                       point_time_config.maximum_pose_gap_sec, 0.15);
    private_node.param("maximum_point_time_distance_sec",
                       point_time_config.maximum_point_time_distance_sec,
                       1.0);
    double minimum_deskewed_fraction = 0.95;
    private_node.param("minimum_deskewed_fraction",
                       minimum_deskewed_fraction, 0.95);
    minimum_deskewed_fraction =
        std::clamp(minimum_deskewed_fraction, 0.0, 1.0);
    std::string preprocess_to_frame = "vehicle_link";
    std::string reference_pose_frame = "vehicle_link";
    private_node.param<std::string>(
        "preprocess_to_frame", preprocess_to_frame, "vehicle_link");
    private_node.param<std::string>(
        "reference_pose_frame", reference_pose_frame, "vehicle_link");
    preprocess_to_frame = normalizeFrameId(preprocess_to_frame);
    reference_pose_frame = normalizeFrameId(reference_pose_frame);
    onion_cloud_preprocessing::CropBounds crop_bounds;
    private_node.param("vehicle_crop_min_x", crop_bounds.min_x, -0.6);
    private_node.param("vehicle_crop_max_x", crop_bounds.max_x, 1.65);
    private_node.param("vehicle_crop_min_y", crop_bounds.min_y, -0.7);
    private_node.param("vehicle_crop_max_y", crop_bounds.max_y, 0.7);
    private_node.param("vehicle_crop_min_z", crop_bounds.min_z, -0.5);
    private_node.param("vehicle_crop_max_z", crop_bounds.max_z, 2.5);
    double tf_timeout_sec = 5.0;
    private_node.param("tf_timeout_sec", tf_timeout_sec, 5.0);
    if (point_time_config.required &&
        !point_time_config.enabled) {
      throw std::runtime_error(
          "require_point_deskew=true but point deskew is disabled");
    }
    if (preprocess_to_frame.empty() ||
        reference_pose_frame != preprocess_to_frame) {
      throw std::runtime_error(
          "reference_pose_frame must equal preprocess_to_frame; "
          "the mapping CSV pose and database point coordinates must "
          "describe the same rigid body frame");
    }
    if (!crop_bounds.valid() || !std::isfinite(tf_timeout_sec) ||
        tf_timeout_sec <= 0.0) {
      throw std::runtime_error(
          "invalid database vehicle crop bounds or TF timeout");
    }

    const std::string bag_path = argv[1];
    const std::string topic = argv[2];
    const std::string pose_path = argv[3];
    const boost::filesystem::path output_root(argv[4]);
    const double keyframe_interval =
        argc >= 6 ? std::stod(argv[5]) : 2.0;
    const double half_window = argc >= 7 ? std::stod(argv[6]) : 1.0;
    if (keyframe_interval <= 0.0 || half_window <= 0.0) {
      throw std::runtime_error(
          "keyframe interval and half window must be positive");
    }

    const auto poses = loadReferencePoses(pose_path);
    const auto frames =
        loadFrames(bag_path, topic, poses, point_time_config,
                   minimum_deskewed_fraction, preprocess_to_frame,
                   crop_bounds, tf_timeout_sec, &transform_buffer);
    const boost::filesystem::path keyframe_root =
        output_root / "keyframes";
    const boost::filesystem::path descriptor_root =
        output_root / "descriptors";
    boost::filesystem::create_directories(keyframe_root);
    boost::filesystem::create_directories(descriptor_root);
    std::ofstream manifest((output_root / "manifest.csv").string());
    if (!manifest) {
      throw std::runtime_error("cannot create database manifest");
    }
    manifest
        << "id,stamp_sec,x,y,z,qx,qy,qz,qw,coarse_pcd,fine_pcd,"
           "source_frame_index,descriptor_file,point_deskewed,"
           "anchor_frame\n";

    std::size_t database_index = 0;
    SCManager scan_context;
    const double first_elapsed =
        frames.front().pose.elapsed_sec + half_window;
    const double last_elapsed =
        frames.back().pose.elapsed_sec - half_window;
    for (double elapsed = first_elapsed; elapsed <= last_elapsed;
         elapsed += keyframe_interval) {
      const std::size_t anchor_index = nearestFrame(frames, elapsed);
      const CloudPtr coarse =
          aggregate(frames, anchor_index, half_window, 0.60F);
      const CloudPtr fine =
          aggregate(frames, anchor_index, half_window, 0.20F);
      std::ostringstream id_stream;
      id_stream << std::setfill('0') << std::setw(6) << database_index;
      const std::string id = id_stream.str();
      const std::string coarse_relative =
          "keyframes/" + id + "_coarse.pcd";
      const std::string fine_relative =
          "keyframes/" + id + "_fine.pcd";
      const std::string descriptor_relative =
          "descriptors/" + id + ".scd";
      if (pcl::io::savePCDFileBinaryCompressed(
              (output_root / coarse_relative).string(), *coarse) != 0 ||
          pcl::io::savePCDFileBinaryCompressed(
              (output_root / fine_relative).string(), *fine) != 0) {
        throw std::runtime_error(
            "failed to save keyframe point clouds " + id);
      }
      auto descriptor_cloud = toScanContextCloud(*coarse);
      const auto descriptor =
          scan_context.makeScancontext(descriptor_cloud);
      onion_relocalization::saveDescriptor(
          (output_root / descriptor_relative).string(), descriptor);
      const auto& pose = frames[anchor_index].pose;
      Eigen::Quaterniond quaternion(pose.map_from_sensor.linear());
      quaternion.normalize();
      manifest << id << "," << std::fixed << std::setprecision(9)
               << pose.stamp_sec << ","
               << pose.map_from_sensor.translation().x() << ","
               << pose.map_from_sensor.translation().y() << ","
               << pose.map_from_sensor.translation().z() << ","
               << quaternion.x() << "," << quaternion.y() << ","
               << quaternion.z() << "," << quaternion.w() << ","
               << coarse_relative << "," << fine_relative << ","
                << anchor_index << "," << descriptor_relative << ","
                << (point_time_config.enabled ? 1 : 0) << ","
                << preprocess_to_frame << "\n";
      ++database_index;
      ROS_INFO("database keyframe %s: elapsed=%.3f coarse=%zu fine=%zu",
               id.c_str(), pose.elapsed_sec, coarse->size(), fine->size());
    }
    manifest.close();
    std::cout << "database_entries=" << database_index << "\n"
              << "manifest="
              << (output_root / "manifest.csv").string() << "\n";
  } catch (const std::exception& error) {
    std::cerr << "database build failed: " << error.what() << "\n";
    return 2;
  }
  return 0;
}
