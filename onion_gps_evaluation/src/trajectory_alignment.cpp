#include "onion_gps_evaluation/trajectory_alignment.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <random>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace onion_gps_evaluation {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySquared =
    kWgs84Flattening * (2.0 - kWgs84Flattening);

bool finitePosition(const PoseSample& sample) {
  return std::isfinite(sample.x) && std::isfinite(sample.y) &&
         std::isfinite(sample.z);
}

double interpolateOptional(double left, double right, double fraction) {
  if (std::isfinite(left) && std::isfinite(right)) {
    return left + fraction * (right - left);
  }
  if (std::isfinite(left)) {
    return left;
  }
  return std::isfinite(right) ? right : kNaN;
}

std::optional<std::pair<PoseSample, double>> interpolatePose(
    const std::vector<PoseSample>& samples, double stamp,
    double max_time_difference_sec, double max_interpolation_gap_sec) {
  if (samples.empty()) {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
      samples.begin(), samples.end(), stamp,
      [](const PoseSample& sample, double value) {
        return sample.stamp < value;
      });
  if (upper != samples.end() && std::abs(upper->stamp - stamp) <= 1.0e-9) {
    return std::make_pair(*upper, std::abs(upper->stamp - stamp));
  }
  if (upper == samples.begin()) {
    const double difference = std::abs(samples.front().stamp - stamp);
    if (difference <= max_time_difference_sec) {
      return std::make_pair(samples.front(), difference);
    }
    return std::nullopt;
  }
  if (upper == samples.end()) {
    const double difference = std::abs(samples.back().stamp - stamp);
    if (difference <= max_time_difference_sec) {
      return std::make_pair(samples.back(), difference);
    }
    return std::nullopt;
  }

  const PoseSample& right = *upper;
  const PoseSample& left = *(upper - 1);
  const double gap = right.stamp - left.stamp;
  const double nearest_difference =
      std::min(stamp - left.stamp, right.stamp - stamp);
  if (gap <= 0.0 || gap > max_interpolation_gap_sec ||
      nearest_difference > max_time_difference_sec) {
    const PoseSample& nearest =
        (stamp - left.stamp <= right.stamp - stamp) ? left : right;
    const double difference = std::abs(nearest.stamp - stamp);
    if (difference <= max_time_difference_sec) {
      return std::make_pair(nearest, difference);
    }
    return std::nullopt;
  }

  const double fraction = (stamp - left.stamp) / gap;
  PoseSample result;
  result.stamp = stamp;
  result.x = left.x + fraction * (right.x - left.x);
  result.y = left.y + fraction * (right.y - left.y);
  result.z = left.z + fraction * (right.z - left.z);
  result.yaw = interpolateAngle(left.yaw, right.yaw, fraction);
  result.latitude =
      interpolateOptional(left.latitude, right.latitude, fraction);
  result.longitude =
      interpolateOptional(left.longitude, right.longitude, fraction);
  result.altitude =
      interpolateOptional(left.altitude, right.altitude, fraction);
  result.source = !left.source.empty() ? left.source : right.source;
  return std::make_pair(result, nearest_difference);
}

Eigen::Vector3d geodeticToEcef(double latitude_deg, double longitude_deg,
                              double altitude_m) {
  const double latitude = latitude_deg * kPi / 180.0;
  const double longitude = longitude_deg * kPi / 180.0;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double radius =
      kWgs84SemiMajorM /
      std::sqrt(1.0 -
                kWgs84EccentricitySquared * sin_latitude * sin_latitude);
  return Eigen::Vector3d(
      (radius + altitude_m) * cos_latitude * std::cos(longitude),
      (radius + altitude_m) * cos_latitude * std::sin(longitude),
      (radius * (1.0 - kWgs84EccentricitySquared) + altitude_m) *
          sin_latitude);
}

std::tuple<double, double, double> fitPlanarSvd(
    const std::vector<Eigen::Vector2d>& source,
    const std::vector<Eigen::Vector2d>& target,
    const std::vector<std::uint8_t>& mask) {
  if (source.size() != target.size() || source.size() != mask.size()) {
    throw std::invalid_argument("planar SVD input sizes differ");
  }
  std::size_t count = 0;
  Eigen::Vector2d source_centroid = Eigen::Vector2d::Zero();
  Eigen::Vector2d target_centroid = Eigen::Vector2d::Zero();
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (mask[index] == 0U) {
      continue;
    }
    source_centroid += source[index];
    target_centroid += target[index];
    ++count;
  }
  if (count < 2U) {
    throw std::invalid_argument("planar SVD needs at least two poses");
  }
  source_centroid /= static_cast<double>(count);
  target_centroid /= static_cast<double>(count);

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (mask[index] == 0U) {
      continue;
    }
    covariance += (source[index] - source_centroid) *
                  (target[index] - target_centroid).transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d rotation = svd.matrixV() * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix2d correction = Eigen::Matrix2d::Identity();
    correction(1, 1) = -1.0;
    rotation = svd.matrixV() * correction * svd.matrixU().transpose();
  }
  const Eigen::Vector2d translation =
      target_centroid - rotation * source_centroid;
  return std::make_tuple(std::atan2(rotation(1, 0), rotation(0, 0)),
                         translation.x(), translation.y());
}

std::optional<std::tuple<double, double, double>> candidateFromPair(
    const std::vector<Eigen::Vector2d>& source,
    const std::vector<Eigen::Vector2d>& target, std::size_t first,
    std::size_t second) {
  const Eigen::Vector2d source_delta = source[second] - source[first];
  const Eigen::Vector2d target_delta = target[second] - target[first];
  if (source_delta.norm() <= 1.0e-6 || target_delta.norm() <= 1.0e-6) {
    return std::nullopt;
  }
  const double yaw =
      wrapAngle(std::atan2(target_delta.y(), target_delta.x()) -
                std::atan2(source_delta.y(), source_delta.x()));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Eigen::Matrix2d rotation;
  rotation << cosine, -sine, sine, cosine;
  const Eigen::Vector2d translation =
      target[first] - rotation * source[first];
  return std::make_tuple(yaw, translation.x(), translation.y());
}

std::vector<double> horizontalResiduals(
    const std::vector<Eigen::Vector2d>& source,
    const std::vector<Eigen::Vector2d>& target, double yaw, double tx,
    double ty) {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Eigen::Matrix2d rotation;
  rotation << cosine, -sine, sine, cosine;
  const Eigen::Vector2d translation(tx, ty);
  std::vector<double> residuals(source.size(), kNaN);
  for (std::size_t index = 0; index < source.size(); ++index) {
    residuals[index] =
        (rotation * source[index] + translation - target[index]).norm();
  }
  return residuals;
}

std::size_t countMask(const std::vector<std::uint8_t>& mask) {
  return static_cast<std::size_t>(
      std::count(mask.begin(), mask.end(), static_cast<std::uint8_t>(1U)));
}

double medianOf(std::vector<double> values) {
  values.erase(
      std::remove_if(values.begin(), values.end(),
                     [](double value) { return !std::isfinite(value); }),
      values.end());
  if (values.empty()) {
    return kNaN;
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 0U) {
    return 0.5 * (values[middle - 1U] + values[middle]);
  }
  return values[middle];
}

double percentile(std::vector<double> values, double fraction) {
  values.erase(
      std::remove_if(values.begin(), values.end(),
                     [](double value) { return !std::isfinite(value); }),
      values.end());
  if (values.empty()) {
    return kNaN;
  }
  std::sort(values.begin(), values.end());
  const double position =
      static_cast<double>(values.size() - 1U) * fraction;
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper) {
    return values[lower];
  }
  const double weight = position - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

std::tuple<double, double, double, double> relativeMotion(
    const PoseSample& start, const PoseSample& end) {
  const double delta_x = end.x - start.x;
  const double delta_y = end.y - start.y;
  const double cosine = std::cos(start.yaw);
  const double sine = std::sin(start.yaw);
  return std::make_tuple(
      cosine * delta_x + sine * delta_y,
      -sine * delta_x + cosine * delta_y, end.z - start.z,
      wrapAngle(end.yaw - start.yaw));
}

}  // namespace

double wrapAngle(double angle) {
  return std::remainder(angle, 2.0 * kPi);
}

double interpolateAngle(double left, double right, double fraction) {
  if (!std::isfinite(left) || !std::isfinite(right)) {
    return kNaN;
  }
  return wrapAngle(left + fraction * wrapAngle(right - left));
}

double quaternionToYaw(double x, double y, double z, double w) {
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm <= 1.0e-12) {
    return kNaN;
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

PoseSample PlanarTransform::apply(const PoseSample& sample) const {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  PoseSample transformed = sample;
  transformed.x = cosine * sample.x - sine * sample.y + tx;
  transformed.y = sine * sample.x + cosine * sample.y + ty;
  transformed.z = sample.z + tz;
  transformed.yaw =
      std::isfinite(sample.yaw) ? wrapAngle(sample.yaw + yaw) : kNaN;
  return transformed;
}

std::vector<MatchedPose> associateTrajectories(
    const std::vector<PoseSample>& estimates,
    const std::vector<PoseSample>& references,
    double max_time_difference_sec, double max_interpolation_gap_sec) {
  std::vector<PoseSample> sorted_estimates = estimates;
  std::vector<PoseSample> sorted_references = references;
  const auto compare_stamp = [](const PoseSample& left,
                                const PoseSample& right) {
    return left.stamp < right.stamp;
  };
  std::sort(sorted_estimates.begin(), sorted_estimates.end(), compare_stamp);
  std::sort(sorted_references.begin(), sorted_references.end(), compare_stamp);

  std::vector<MatchedPose> matches;
  matches.reserve(sorted_estimates.size());
  for (const PoseSample& estimate : sorted_estimates) {
    const auto interpolated =
        interpolatePose(sorted_references, estimate.stamp,
                        max_time_difference_sec, max_interpolation_gap_sec);
    if (!interpolated.has_value()) {
      continue;
    }
    matches.push_back(
        MatchedPose{estimate, interpolated->first, interpolated->second});
  }
  return matches;
}

PoseSample geodeticToEnu(double latitude_deg, double longitude_deg,
                         double altitude_m, double reference_latitude_deg,
                         double reference_longitude_deg,
                         double reference_altitude_m) {
  const Eigen::Vector3d point =
      geodeticToEcef(latitude_deg, longitude_deg, altitude_m);
  const Eigen::Vector3d origin =
      geodeticToEcef(reference_latitude_deg, reference_longitude_deg,
                     reference_altitude_m);
  const Eigen::Vector3d difference = point - origin;
  const double latitude = reference_latitude_deg * kPi / 180.0;
  const double longitude = reference_longitude_deg * kPi / 180.0;
  Eigen::Matrix3d rotation;
  rotation << -std::sin(longitude), std::cos(longitude), 0.0,
      -std::sin(latitude) * std::cos(longitude),
      -std::sin(latitude) * std::sin(longitude), std::cos(latitude),
      std::cos(latitude) * std::cos(longitude),
      std::cos(latitude) * std::sin(longitude), std::sin(latitude);
  const Eigen::Vector3d enu = rotation * difference;
  PoseSample result;
  result.x = enu.x();
  result.y = enu.y();
  result.z = enu.z();
  result.latitude = latitude_deg;
  result.longitude = longitude_deg;
  result.altitude = altitude_m;
  return result;
}

AlignmentResult estimatePlanarAlignment(
    const std::vector<PoseSample>& source_samples,
    const std::vector<PoseSample>& target_samples, std::size_t minimum_inliers,
    double minimum_baseline_m, double outlier_threshold_m,
    int ransac_iterations, std::uint32_t random_seed) {
  if (source_samples.size() != target_samples.size()) {
    throw std::invalid_argument("source and target sample counts differ");
  }
  if (source_samples.size() < minimum_inliers) {
    throw std::invalid_argument("not enough registration poses");
  }
  if (outlier_threshold_m <= 0.0) {
    throw std::invalid_argument("outlier threshold must be positive");
  }

  std::vector<Eigen::Vector2d> source_xy;
  std::vector<Eigen::Vector2d> target_xy;
  std::vector<double> source_z;
  std::vector<double> target_z;
  std::vector<std::size_t> finite_indices;
  source_xy.reserve(source_samples.size());
  target_xy.reserve(source_samples.size());
  for (std::size_t index = 0; index < source_samples.size(); ++index) {
    if (!finitePosition(source_samples[index]) ||
        !finitePosition(target_samples[index])) {
      continue;
    }
    source_xy.emplace_back(source_samples[index].x, source_samples[index].y);
    target_xy.emplace_back(target_samples[index].x, target_samples[index].y);
    source_z.push_back(source_samples[index].z);
    target_z.push_back(target_samples[index].z);
    finite_indices.push_back(index);
  }
  if (source_xy.size() < minimum_inliers) {
    throw std::invalid_argument("not enough finite registration poses");
  }

  double source_baseline_m = 0.0;
  for (const Eigen::Vector2d& point : source_xy) {
    source_baseline_m =
        std::max(source_baseline_m, (point - source_xy.front()).norm());
  }
  if (source_baseline_m < minimum_baseline_m) {
    throw std::invalid_argument("registration baseline is below minimum");
  }

  std::mt19937 random(random_seed);
  std::uniform_int_distribution<std::size_t> distribution(
      0U, source_xy.size() - 1U);
  std::vector<std::uint8_t> best_mask(source_xy.size(), 0U);
  double best_median = std::numeric_limits<double>::infinity();
  const int iterations = std::max(1, ransac_iterations);
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const std::size_t first = distribution(random);
    std::size_t second = distribution(random);
    while (second == first) {
      second = distribution(random);
    }
    const auto candidate =
        candidateFromPair(source_xy, target_xy, first, second);
    if (!candidate.has_value()) {
      continue;
    }
    const auto [yaw, tx, ty] = *candidate;
    const std::vector<double> residuals =
        horizontalResiduals(source_xy, target_xy, yaw, tx, ty);
    std::vector<std::uint8_t> mask(source_xy.size(), 0U);
    std::vector<double> inlier_residuals;
    for (std::size_t index = 0; index < residuals.size(); ++index) {
      if (residuals[index] <= outlier_threshold_m) {
        mask[index] = 1U;
        inlier_residuals.push_back(residuals[index]);
      }
    }
    const std::size_t inlier_count = inlier_residuals.size();
    const double candidate_median = medianOf(inlier_residuals);
    if (inlier_count > countMask(best_mask) ||
        (inlier_count == countMask(best_mask) &&
         candidate_median < best_median)) {
      best_mask = std::move(mask);
      best_median = candidate_median;
    }
  }

  if (countMask(best_mask) < minimum_inliers) {
    std::vector<std::uint8_t> all_mask(source_xy.size(), 1U);
    const auto [yaw, tx, ty] =
        fitPlanarSvd(source_xy, target_xy, all_mask);
    const std::vector<double> residuals =
        horizontalResiduals(source_xy, target_xy, yaw, tx, ty);
    for (std::size_t index = 0; index < residuals.size(); ++index) {
      best_mask[index] =
          residuals[index] <= outlier_threshold_m ? 1U : 0U;
    }
  }
  if (countMask(best_mask) < minimum_inliers) {
    throw std::runtime_error(
        "robust registration retained fewer than minimum poses");
  }

  for (int refinement = 0; refinement < 5; ++refinement) {
    const auto [yaw, tx, ty] =
        fitPlanarSvd(source_xy, target_xy, best_mask);
    const std::vector<double> residuals =
        horizontalResiduals(source_xy, target_xy, yaw, tx, ty);
    std::vector<std::uint8_t> refined_mask(source_xy.size(), 0U);
    for (std::size_t index = 0; index < residuals.size(); ++index) {
      refined_mask[index] =
          residuals[index] <= outlier_threshold_m ? 1U : 0U;
    }
    if (refined_mask == best_mask ||
        countMask(refined_mask) < minimum_inliers) {
      break;
    }
    best_mask = std::move(refined_mask);
  }

  const auto [yaw, tx, ty] =
      fitPlanarSvd(source_xy, target_xy, best_mask);
  const std::vector<double> residuals =
      horizontalResiduals(source_xy, target_xy, yaw, tx, ty);
  std::vector<double> height_offsets;
  double squared_error_sum = 0.0;
  for (std::size_t index = 0; index < best_mask.size(); ++index) {
    if (best_mask[index] == 0U) {
      continue;
    }
    height_offsets.push_back(target_z[index] - source_z[index]);
    squared_error_sum += residuals[index] * residuals[index];
  }

  AlignmentResult result;
  result.transform =
      PlanarTransform{yaw, tx, ty, medianOf(height_offsets)};
  result.horizontal_rmse_m =
      std::sqrt(squared_error_sum /
                static_cast<double>(height_offsets.size()));
  result.source_baseline_m = source_baseline_m;
  result.inlier_mask.assign(source_samples.size(), 0U);
  result.horizontal_residuals_m.assign(source_samples.size(), kNaN);
  for (std::size_t index = 0; index < finite_indices.size(); ++index) {
    result.inlier_mask[finite_indices[index]] = best_mask[index];
    result.horizontal_residuals_m[finite_indices[index]] = residuals[index];
  }
  return result;
}

Distribution calculateDistribution(const std::vector<double>& values) {
  std::vector<double> finite_values;
  finite_values.reserve(values.size());
  std::copy_if(values.begin(), values.end(),
               std::back_inserter(finite_values),
               [](double value) { return std::isfinite(value); });
  Distribution result;
  result.count = finite_values.size();
  if (finite_values.empty()) {
    return result;
  }
  result.min =
      *std::min_element(finite_values.begin(), finite_values.end());
  result.max =
      *std::max_element(finite_values.begin(), finite_values.end());
  result.mean =
      std::accumulate(finite_values.begin(), finite_values.end(), 0.0) /
      static_cast<double>(finite_values.size());
  result.median = medianOf(finite_values);
  result.p95 = percentile(finite_values, 0.95);
  double squared_sum = 0.0;
  for (double value : finite_values) {
    squared_sum += value * value;
  }
  result.rmse =
      std::sqrt(squared_sum / static_cast<double>(finite_values.size()));
  return result;
}

std::vector<ErrorRow> buildErrorRows(
    const std::vector<MatchedPose>& matches,
    const PlanarTransform& transform) {
  std::vector<ErrorRow> rows;
  rows.reserve(matches.size());
  for (const MatchedPose& match : matches) {
    const PoseSample aligned = transform.apply(match.estimate);
    const double delta_x = aligned.x - match.reference.x;
    const double delta_y = aligned.y - match.reference.y;
    const double delta_z = aligned.z - match.reference.z;
    ErrorRow row;
    row.stamp_sec = match.estimate.stamp;
    row.time_difference_sec = match.time_difference_sec;
    row.latitude_deg = match.reference.latitude;
    row.longitude_deg = match.reference.longitude;
    row.altitude_m = match.reference.altitude;
    row.gps_x = match.reference.x;
    row.gps_y = match.reference.y;
    row.gps_z = match.reference.z;
    row.gps_yaw_rad = match.reference.yaw;
    row.onion_x = match.estimate.x;
    row.onion_y = match.estimate.y;
    row.onion_z = match.estimate.z;
    row.onion_yaw_rad = match.estimate.yaw;
    row.aligned_onion_x = aligned.x;
    row.aligned_onion_y = aligned.y;
    row.aligned_onion_z = aligned.z;
    row.aligned_onion_yaw_rad = aligned.yaw;
    row.horizontal_error_m = std::hypot(delta_x, delta_y);
    row.vertical_error_m = std::abs(delta_z);
    row.error_3d_m =
        std::sqrt(delta_x * delta_x + delta_y * delta_y +
                  delta_z * delta_z);
    if (std::isfinite(aligned.yaw) &&
        std::isfinite(match.reference.yaw)) {
      row.yaw_error_deg =
          std::abs(wrapAngle(aligned.yaw - match.reference.yaw)) *
          180.0 / kPi;
    }
    rows.push_back(row);
  }
  return rows;
}

RpeResult computeRpe(const std::vector<MatchedPose>& matches,
                     const PlanarTransform& transform, double delta_sec) {
  RpeResult result;
  result.delta_sec = delta_sec;
  if (delta_sec <= 0.0 || matches.size() < 2U) {
    return result;
  }
  std::vector<MatchedPose> ordered = matches;
  std::sort(ordered.begin(), ordered.end(),
            [](const MatchedPose& left, const MatchedPose& right) {
              return left.estimate.stamp < right.estimate.stamp;
            });
  std::vector<double> stamps;
  stamps.reserve(ordered.size());
  for (const MatchedPose& match : ordered) {
    stamps.push_back(match.estimate.stamp);
  }

  const double tolerance = std::max(0.20, 0.25 * delta_sec);
  std::vector<double> translation_errors;
  std::vector<double> yaw_errors;
  for (std::size_t index = 0; index + 1U < ordered.size(); ++index) {
    const double target_stamp = stamps[index] + delta_sec;
    const auto upper = std::lower_bound(stamps.begin() + index + 1U,
                                        stamps.end(), target_stamp);
    std::vector<std::size_t> candidates;
    if (upper != stamps.end()) {
      candidates.push_back(
          static_cast<std::size_t>(upper - stamps.begin()));
    }
    if (upper != stamps.begin() + index + 1U) {
      candidates.push_back(
          static_cast<std::size_t>((upper - stamps.begin()) - 1));
    }
    if (candidates.empty()) {
      continue;
    }
    const std::size_t target_index = *std::min_element(
        candidates.begin(), candidates.end(),
        [&](std::size_t left, std::size_t right) {
          return std::abs(stamps[left] - target_stamp) <
                 std::abs(stamps[right] - target_stamp);
        });
    if (std::abs(stamps[target_index] - target_stamp) > tolerance) {
      continue;
    }

    const PoseSample start_estimate =
        transform.apply(ordered[index].estimate);
    const PoseSample end_estimate =
        transform.apply(ordered[target_index].estimate);
    const PoseSample& start_reference = ordered[index].reference;
    const PoseSample& end_reference = ordered[target_index].reference;
    const bool orientations_available =
        std::isfinite(start_estimate.yaw) &&
        std::isfinite(end_estimate.yaw) &&
        std::isfinite(start_reference.yaw) &&
        std::isfinite(end_reference.yaw);

    double estimate_dx = end_estimate.x - start_estimate.x;
    double estimate_dy = end_estimate.y - start_estimate.y;
    double estimate_dz = end_estimate.z - start_estimate.z;
    double reference_dx = end_reference.x - start_reference.x;
    double reference_dy = end_reference.y - start_reference.y;
    double reference_dz = end_reference.z - start_reference.z;
    double estimate_dyaw = kNaN;
    double reference_dyaw = kNaN;
    if (orientations_available) {
      std::tie(estimate_dx, estimate_dy, estimate_dz, estimate_dyaw) =
          relativeMotion(start_estimate, end_estimate);
      std::tie(reference_dx, reference_dy, reference_dz, reference_dyaw) =
          relativeMotion(start_reference, end_reference);
    }
    translation_errors.push_back(
        std::sqrt(std::pow(estimate_dx - reference_dx, 2.0) +
                  std::pow(estimate_dy - reference_dy, 2.0) +
                  std::pow(estimate_dz - reference_dz, 2.0)));
    if (orientations_available) {
      yaw_errors.push_back(
          std::abs(wrapAngle(estimate_dyaw - reference_dyaw)) * 180.0 /
          kPi);
    }
  }
  result.translation_error_m =
      calculateDistribution(translation_errors);
  result.yaw_error_deg = calculateDistribution(yaw_errors);
  return result;
}

}  // namespace onion_gps_evaluation
