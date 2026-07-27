#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace onion_relocalization {

struct TimedPose {
  double stamp_sec = 0.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

struct PointTimeConfig {
  bool enabled = false;
  bool required = false;
  std::string primary_field = "t_sec";
  double primary_scale = 1.0;
  std::string secondary_field = "t_usec";
  double secondary_scale = 1.0e-6;
  bool time_is_offset = false;
  double maximum_pose_gap_sec = 0.15;
  double maximum_point_time_distance_sec = 1.0;
};

struct DeskewResult {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{
      new pcl::PointCloud<pcl::PointXYZ>};
  std::size_t valid_points = 0;
  std::size_t invalid_points = 0;
  double minimum_point_stamp_sec =
      std::numeric_limits<double>::infinity();
  double maximum_point_stamp_sec =
      -std::numeric_limits<double>::infinity();
  std::string error;

  bool ok() const { return error.empty() && valid_points > 0; }
};

inline const sensor_msgs::PointField* findField(
    const sensor_msgs::PointCloud2& message,
    const std::string& name) {
  const auto iterator =
      std::find_if(message.fields.begin(), message.fields.end(),
                   [&name](const sensor_msgs::PointField& field) {
                     return field.name == name;
                   });
  return iterator == message.fields.end() ? nullptr : &*iterator;
}

template <typename T>
inline T byteSwapped(T value) {
  T output{};
  const auto* source = reinterpret_cast<const std::uint8_t*>(&value);
  auto* target = reinterpret_cast<std::uint8_t*>(&output);
  std::reverse_copy(source, source + sizeof(T), target);
  return output;
}

template <typename T>
inline bool readValue(const sensor_msgs::PointCloud2& message,
                      std::size_t offset,
                      double* value) {
  if (offset + sizeof(T) > message.data.size()) {
    return false;
  }
  T raw{};
  std::memcpy(&raw, message.data.data() + offset, sizeof(T));
  if (message.is_bigendian) {
    raw = byteSwapped(raw);
  }
  *value = static_cast<double>(raw);
  return true;
}

inline bool readField(const sensor_msgs::PointCloud2& message,
                      const sensor_msgs::PointField& field,
                      std::size_t point_offset,
                      double* value) {
  const std::size_t offset = point_offset + field.offset;
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8:
      return readValue<std::int8_t>(message, offset, value);
    case sensor_msgs::PointField::UINT8:
      return readValue<std::uint8_t>(message, offset, value);
    case sensor_msgs::PointField::INT16:
      return readValue<std::int16_t>(message, offset, value);
    case sensor_msgs::PointField::UINT16:
      return readValue<std::uint16_t>(message, offset, value);
    case sensor_msgs::PointField::INT32:
      return readValue<std::int32_t>(message, offset, value);
    case sensor_msgs::PointField::UINT32:
      return readValue<std::uint32_t>(message, offset, value);
    case sensor_msgs::PointField::FLOAT32:
      return readValue<float>(message, offset, value);
    case sensor_msgs::PointField::FLOAT64:
      return readValue<double>(message, offset, value);
    default:
      return false;
  }
}

inline bool interpolatePose(const std::vector<TimedPose>& poses,
                            double stamp_sec,
                            double maximum_gap_sec,
                            Eigen::Isometry3d* pose) {
  if (poses.empty() || !std::isfinite(stamp_sec)) {
    return false;
  }
  const auto upper =
      std::lower_bound(poses.begin(), poses.end(), stamp_sec,
                       [](const TimedPose& sample, double value) {
                         return sample.stamp_sec < value;
                       });
  if (upper != poses.end() &&
      std::abs(upper->stamp_sec - stamp_sec) <= 1.0e-6) {
    *pose = upper->pose;
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
  Eigen::Quaterniond lower_rotation(lower->pose.linear());
  Eigen::Quaterniond upper_rotation(upper->pose.linear());
  lower_rotation.normalize();
  upper_rotation.normalize();
  Eigen::Isometry3d interpolated = Eigen::Isometry3d::Identity();
  interpolated.translation() =
      (1.0 - alpha) * lower->pose.translation() +
      alpha * upper->pose.translation();
  interpolated.linear() =
      lower_rotation.slerp(alpha, upper_rotation).normalized()
          .toRotationMatrix();
  *pose = interpolated;
  return true;
}

inline bool pointTimeRange(const sensor_msgs::PointCloud2& message,
                           const PointTimeConfig& config,
                           double* minimum_stamp_sec,
                           double* maximum_stamp_sec,
                           std::string* error) {
  const auto* primary = findField(message, config.primary_field);
  const auto* secondary =
      config.secondary_field.empty()
          ? nullptr
          : findField(message, config.secondary_field);
  if (!primary ||
      (!config.secondary_field.empty() && secondary == nullptr)) {
    if (error) {
      *error = "point time field missing: " + config.primary_field +
               (config.secondary_field.empty()
                    ? ""
                    : " + " + config.secondary_field);
    }
    return false;
  }

  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (std::size_t row = 0; row < message.height; ++row) {
    for (std::size_t column = 0; column < message.width; ++column) {
      const std::size_t point_offset =
          row * message.row_step + column * message.point_step;
      double primary_value = 0.0;
      double secondary_value = 0.0;
      if (!readField(message, *primary, point_offset, &primary_value) ||
          (secondary &&
           !readField(message, *secondary, point_offset,
                      &secondary_value))) {
        continue;
      }
      double stamp = primary_value * config.primary_scale +
                     secondary_value * config.secondary_scale;
      if (config.time_is_offset) {
        stamp += message.header.stamp.toSec();
      }
      if (std::isfinite(stamp)) {
        minimum = std::min(minimum, stamp);
        maximum = std::max(maximum, stamp);
      }
    }
  }
  if (!std::isfinite(minimum) || !std::isfinite(maximum)) {
    if (error) {
      *error = "point time fields contain no finite values";
    }
    return false;
  }
  const double header_stamp = message.header.stamp.toSec();
  if (std::abs(minimum - header_stamp) >
          config.maximum_point_time_distance_sec ||
      std::abs(maximum - header_stamp) >
          config.maximum_point_time_distance_sec) {
    if (error) {
      *error = "point times are not compatible with the message stamp";
    }
    return false;
  }
  *minimum_stamp_sec = minimum;
  *maximum_stamp_sec = maximum;
  return true;
}

inline DeskewResult deskewPointCloud(
    const sensor_msgs::PointCloud2& message,
    const PointTimeConfig& config,
    const std::vector<TimedPose>& poses) {
  DeskewResult output;
  const auto* x_field = findField(message, "x");
  const auto* y_field = findField(message, "y");
  const auto* z_field = findField(message, "z");
  const auto* primary = findField(message, config.primary_field);
  const auto* secondary =
      config.secondary_field.empty()
          ? nullptr
          : findField(message, config.secondary_field);
  if (!x_field || !y_field || !z_field) {
    output.error = "PointCloud2 is missing x/y/z fields";
    return output;
  }
  if (!primary ||
      (!config.secondary_field.empty() && secondary == nullptr)) {
    output.error = "point time field missing: " +
                   config.primary_field +
                   (config.secondary_field.empty()
                        ? ""
                        : " + " + config.secondary_field);
    return output;
  }

  Eigen::Isometry3d pose_at_anchor = Eigen::Isometry3d::Identity();
  if (!interpolatePose(poses, message.header.stamp.toSec(),
                       config.maximum_pose_gap_sec, &pose_at_anchor)) {
    output.error = "pose interpolation failed at cloud header stamp";
    return output;
  }
  const Eigen::Isometry3d anchor_from_world = pose_at_anchor.inverse();
  output.cloud->reserve(
      static_cast<std::size_t>(message.width) * message.height);
  for (std::size_t row = 0; row < message.height; ++row) {
    for (std::size_t column = 0; column < message.width; ++column) {
      const std::size_t point_offset =
          row * message.row_step + column * message.point_step;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double primary_value = 0.0;
      double secondary_value = 0.0;
      if (!readField(message, *x_field, point_offset, &x) ||
          !readField(message, *y_field, point_offset, &y) ||
          !readField(message, *z_field, point_offset, &z) ||
          !readField(message, *primary, point_offset, &primary_value) ||
          (secondary &&
           !readField(message, *secondary, point_offset,
                      &secondary_value)) ||
          !std::isfinite(x) || !std::isfinite(y) ||
          !std::isfinite(z)) {
        ++output.invalid_points;
        continue;
      }
      double point_stamp =
          primary_value * config.primary_scale +
          secondary_value * config.secondary_scale;
      if (config.time_is_offset) {
        point_stamp += message.header.stamp.toSec();
      }
      if (!std::isfinite(point_stamp) ||
          std::abs(point_stamp - message.header.stamp.toSec()) >
              config.maximum_point_time_distance_sec) {
        ++output.invalid_points;
        continue;
      }
      Eigen::Isometry3d pose_at_point = Eigen::Isometry3d::Identity();
      if (!interpolatePose(poses, point_stamp,
                           config.maximum_pose_gap_sec,
                           &pose_at_point)) {
        ++output.invalid_points;
        continue;
      }
      const Eigen::Vector3d corrected =
          anchor_from_world * pose_at_point * Eigen::Vector3d(x, y, z);
      pcl::PointXYZ point;
      point.x = static_cast<float>(corrected.x());
      point.y = static_cast<float>(corrected.y());
      point.z = static_cast<float>(corrected.z());
      output.cloud->push_back(point);
      ++output.valid_points;
      output.minimum_point_stamp_sec =
          std::min(output.minimum_point_stamp_sec, point_stamp);
      output.maximum_point_stamp_sec =
          std::max(output.maximum_point_stamp_sec, point_stamp);
    }
  }
  if (output.valid_points == 0) {
    output.error = "no point could be deskewed with the pose trajectory";
  }
  return output;
}

}  // namespace onion_relocalization
