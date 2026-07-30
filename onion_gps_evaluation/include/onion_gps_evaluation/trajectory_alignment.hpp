#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace onion_gps_evaluation {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct PoseSample {
  double stamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = kNaN;
  double latitude = kNaN;
  double longitude = kNaN;
  double altitude = kNaN;
  std::string source;
};

struct MatchedPose {
  PoseSample estimate;
  PoseSample reference;
  double time_difference_sec = 0.0;
};

struct PlanarTransform {
  double yaw = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;

  PoseSample apply(const PoseSample& sample) const;
};

struct AlignmentResult {
  PlanarTransform transform;
  std::vector<std::uint8_t> inlier_mask;
  std::vector<double> horizontal_residuals_m;
  double horizontal_rmse_m = kNaN;
  double source_baseline_m = 0.0;
};

struct Distribution {
  std::size_t count = 0;
  double min = kNaN;
  double mean = kNaN;
  double median = kNaN;
  double p95 = kNaN;
  double max = kNaN;
  double rmse = kNaN;
};

struct ErrorRow {
  std::string phase;
  double stamp_sec = 0.0;
  double time_difference_sec = 0.0;
  double latitude_deg = kNaN;
  double longitude_deg = kNaN;
  double altitude_m = kNaN;
  double gps_x = 0.0;
  double gps_y = 0.0;
  double gps_z = 0.0;
  double gps_yaw_rad = kNaN;
  double onion_x = 0.0;
  double onion_y = 0.0;
  double onion_z = 0.0;
  double onion_yaw_rad = kNaN;
  double aligned_onion_x = 0.0;
  double aligned_onion_y = 0.0;
  double aligned_onion_z = 0.0;
  double aligned_onion_yaw_rad = kNaN;
  double horizontal_error_m = 0.0;
  double vertical_error_m = 0.0;
  double error_3d_m = 0.0;
  double yaw_error_deg = kNaN;
};

struct RpeResult {
  double delta_sec = 0.0;
  Distribution translation_error_m;
  Distribution yaw_error_deg;
};

double wrapAngle(double angle);
double interpolateAngle(double left, double right, double fraction);
double quaternionToYaw(double x, double y, double z, double w);

std::vector<MatchedPose> associateTrajectories(
    const std::vector<PoseSample>& estimates,
    const std::vector<PoseSample>& references,
    double max_time_difference_sec,
    double max_interpolation_gap_sec);

PoseSample geodeticToEnu(double latitude_deg, double longitude_deg,
                         double altitude_m, double reference_latitude_deg,
                         double reference_longitude_deg,
                         double reference_altitude_m);

AlignmentResult estimatePlanarAlignment(
    const std::vector<PoseSample>& source_samples,
    const std::vector<PoseSample>& target_samples, std::size_t minimum_inliers,
    double minimum_baseline_m, double outlier_threshold_m,
    int ransac_iterations, std::uint32_t random_seed);

Distribution calculateDistribution(const std::vector<double>& values);

std::vector<ErrorRow> buildErrorRows(
    const std::vector<MatchedPose>& matches,
    const PlanarTransform& transform);

RpeResult computeRpe(const std::vector<MatchedPose>& matches,
                     const PlanarTransform& transform, double delta_sec);

}  // namespace onion_gps_evaluation
