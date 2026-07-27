#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace onion_cloud_preprocessing {

struct CropBounds {
  double min_x = -1.0;
  double max_x = 1.0;
  double min_y = -1.0;
  double max_y = 1.0;
  double min_z = -1.0;
  double max_z = 2.0;

  bool valid() const {
    return std::isfinite(min_x) && std::isfinite(max_x) &&
           std::isfinite(min_y) && std::isfinite(max_y) &&
           std::isfinite(min_z) && std::isfinite(max_z) &&
           min_x < max_x && min_y < max_y && min_z < max_z;
  }

  bool contains(const Eigen::Vector3d& point) const {
    return point.x() >= min_x && point.x() <= max_x &&
           point.y() >= min_y && point.y() <= max_y &&
           point.z() >= min_z && point.z() <= max_z;
  }
};

struct CropResult {
  sensor_msgs::PointCloud2 sensor_cloud;
  sensor_msgs::PointCloud2 vehicle_cloud;
  std::size_t input_points = 0;
  std::size_t finite_points = 0;
  std::size_t removed_vehicle_points = 0;
  std::size_t output_points = 0;
};

inline const sensor_msgs::PointField* FindField(
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
inline T ByteSwap(T value) {
  T output{};
  const auto* source = reinterpret_cast<const std::uint8_t*>(&value);
  auto* target = reinterpret_cast<std::uint8_t*>(&output);
  std::reverse_copy(source, source + sizeof(T), target);
  return output;
}

template <typename T>
inline T ReadScalar(const std::uint8_t* data, bool big_endian) {
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return big_endian ? ByteSwap(value) : value;
}

template <typename T>
inline void WriteScalar(T value, bool big_endian, std::uint8_t* data) {
  if (big_endian) value = ByteSwap(value);
  std::memcpy(data, &value, sizeof(T));
}

inline double ReadCoordinate(const sensor_msgs::PointField& field,
                             const std::uint8_t* point_data,
                             bool big_endian) {
  switch (field.datatype) {
    case sensor_msgs::PointField::FLOAT32:
      return static_cast<double>(
          ReadScalar<float>(point_data + field.offset, big_endian));
    case sensor_msgs::PointField::FLOAT64:
      return ReadScalar<double>(point_data + field.offset, big_endian);
    default:
      throw std::runtime_error(
          "PointCloud2 x/y/z fields must be FLOAT32 or FLOAT64");
  }
}

inline void WriteCoordinate(const sensor_msgs::PointField& field,
                            double value, bool big_endian,
                            std::uint8_t* point_data) {
  switch (field.datatype) {
    case sensor_msgs::PointField::FLOAT32:
      WriteScalar<float>(static_cast<float>(value), big_endian,
                         point_data + field.offset);
      return;
    case sensor_msgs::PointField::FLOAT64:
      WriteScalar<double>(value, big_endian,
                          point_data + field.offset);
      return;
    default:
      throw std::runtime_error(
          "PointCloud2 x/y/z fields must be FLOAT32 or FLOAT64");
  }
}

inline sensor_msgs::PointCloud2 EmptyLike(
    const sensor_msgs::PointCloud2& input) {
  sensor_msgs::PointCloud2 output;
  output.header = input.header;
  output.height = 1;
  output.width = 0;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = 0;
  output.is_dense = true;
  return output;
}

inline CropResult CropAndTransform(
    const sensor_msgs::PointCloud2& input,
    const Eigen::Isometry3d& vehicle_from_sensor,
    const CropBounds& bounds, bool publish_sensor_cloud = true,
    bool publish_vehicle_cloud = true) {
  if (!bounds.valid()) {
    throw std::runtime_error(
        "vehicle crop bounds must be finite and min < max");
  }
  if (!vehicle_from_sensor.matrix().allFinite()) {
    throw std::runtime_error(
        "vehicle_from_sensor contains a non-finite value");
  }
  if (input.point_step == 0 || input.height == 0 || input.width == 0) {
    throw std::runtime_error("PointCloud2 has no point records");
  }

  const auto* x_field = FindField(input, "x");
  const auto* y_field = FindField(input, "y");
  const auto* z_field = FindField(input, "z");
  if (!x_field || !y_field || !z_field) {
    throw std::runtime_error("PointCloud2 is missing x/y/z fields");
  }
  const auto field_fits = [&input](
                              const sensor_msgs::PointField& field) {
    const std::size_t scalar_size =
        field.datatype == sensor_msgs::PointField::FLOAT32
            ? sizeof(float)
            : field.datatype == sensor_msgs::PointField::FLOAT64
                  ? sizeof(double)
                  : 0;
    return field.count >= 1 && scalar_size > 0 &&
           static_cast<std::size_t>(field.offset) + scalar_size <=
               input.point_step;
  };
  if (!field_fits(*x_field) || !field_fits(*y_field) ||
      !field_fits(*z_field)) {
    throw std::runtime_error(
        "PointCloud2 x/y/z field layout is unsupported");
  }

  CropResult result;
  result.sensor_cloud = EmptyLike(input);
  result.vehicle_cloud = EmptyLike(input);
  result.input_points =
      static_cast<std::size_t>(input.width) * input.height;
  if (publish_sensor_cloud) {
    result.sensor_cloud.data.resize(result.input_points *
                                    input.point_step);
  }
  if (publish_vehicle_cloud) {
    result.vehicle_cloud.data.resize(result.input_points *
                                     input.point_step);
  }

  for (std::size_t row = 0; row < input.height; ++row) {
    for (std::size_t column = 0; column < input.width; ++column) {
      const std::size_t source_offset =
          row * input.row_step + column * input.point_step;
      if (source_offset + input.point_step > input.data.size()) {
        throw std::runtime_error(
            "PointCloud2 row_step/point_step exceeds data size");
      }
      const auto* source = input.data.data() + source_offset;
      const Eigen::Vector3d sensor_point(
          ReadCoordinate(*x_field, source, input.is_bigendian),
          ReadCoordinate(*y_field, source, input.is_bigendian),
          ReadCoordinate(*z_field, source, input.is_bigendian));
      if (!sensor_point.allFinite()) continue;
      ++result.finite_points;

      const Eigen::Vector3d vehicle_point =
          vehicle_from_sensor * sensor_point;
      if (bounds.contains(vehicle_point)) {
        ++result.removed_vehicle_points;
        continue;
      }

      const std::size_t target_offset =
          result.output_points * input.point_step;
      if (publish_sensor_cloud) {
        std::memcpy(result.sensor_cloud.data.data() + target_offset,
                    source, input.point_step);
      }
      if (publish_vehicle_cloud) {
        auto* target =
            result.vehicle_cloud.data.data() + target_offset;
        std::memcpy(target, source, input.point_step);
        WriteCoordinate(*x_field, vehicle_point.x(),
                        input.is_bigendian, target);
        WriteCoordinate(*y_field, vehicle_point.y(),
                        input.is_bigendian, target);
        WriteCoordinate(*z_field, vehicle_point.z(),
                        input.is_bigendian, target);
      }
      ++result.output_points;
    }
  }

  const auto finalize = [&input, &result](
                            sensor_msgs::PointCloud2* output) {
    output->width = static_cast<std::uint32_t>(result.output_points);
    output->row_step = output->width * input.point_step;
    output->data.resize(output->row_step);
  };
  if (publish_sensor_cloud) finalize(&result.sensor_cloud);
  if (publish_vehicle_cloud) finalize(&result.vehicle_cloud);
  return result;
}

}  // namespace onion_cloud_preprocessing
