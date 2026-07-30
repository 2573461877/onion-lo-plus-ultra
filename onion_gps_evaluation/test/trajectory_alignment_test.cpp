#include "onion_gps_evaluation/trajectory_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace onion_gps_evaluation {
namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(TrajectoryAlignment, ConvertsWgs84ToLocalEnuAxes) {
  const PoseSample origin =
      geodeticToEnu(39.9, 116.3, 50.0, 39.9, 116.3, 50.0);
  EXPECT_NEAR(origin.x, 0.0, 1.0e-6);
  EXPECT_NEAR(origin.y, 0.0, 1.0e-6);
  EXPECT_NEAR(origin.z, 0.0, 1.0e-6);

  const PoseSample north =
      geodeticToEnu(39.90001, 116.3, 50.0, 39.9, 116.3, 50.0);
  const PoseSample east =
      geodeticToEnu(39.9, 116.30001, 50.0, 39.9, 116.3, 50.0);
  EXPECT_GT(north.y, 1.0);
  EXPECT_LT(std::abs(north.x), 0.01);
  EXPECT_GT(east.x, 0.8);
  EXPECT_LT(std::abs(east.y), 0.01);
}

TEST(TrajectoryAlignment, InterpolatesReferenceByTimestamp) {
  std::vector<PoseSample> references(2);
  references[0] = PoseSample{0.0, 0.0, 0.0, 0.0, 0.0};
  references[1] =
      PoseSample{0.1, 1.0, 2.0, 0.2, 10.0 * kPi / 180.0};
  const std::vector<PoseSample> estimates{
      PoseSample{0.05, 0.5, 1.0, 0.1, 5.0 * kPi / 180.0}};
  const std::vector<MatchedPose> matches =
      associateTrajectories(estimates, references, 0.06, 0.2);
  ASSERT_EQ(matches.size(), 1U);
  EXPECT_NEAR(matches.front().reference.x, 0.5, 1.0e-12);
  EXPECT_NEAR(matches.front().reference.y, 1.0, 1.0e-12);
  EXPECT_NEAR(matches.front().reference.yaw, 5.0 * kPi / 180.0,
              1.0e-12);
  EXPECT_NEAR(matches.front().time_difference_sec, 0.05, 1.0e-12);
}

TEST(TrajectoryAlignment, RobustRegistrationRejectsOutliers) {
  const PlanarTransform expected{27.0 * kPi / 180.0, 500000.0,
                                 4400000.0, 52.0};
  std::vector<PoseSample> source;
  std::vector<PoseSample> target;
  for (int index = 0; index < 120; ++index) {
    PoseSample sample;
    sample.stamp = 0.1 * index;
    sample.x = 0.25 * index;
    sample.y = 4.0 * std::sin(0.08 * index);
    sample.z = 0.1 * std::cos(0.05 * index);
    sample.yaw = 0.2 * std::sin(0.03 * index);
    PoseSample transformed = expected.apply(sample);
    if (index % 17 == 0) {
      transformed.x += 8.0;
      transformed.y -= 5.0;
    }
    source.push_back(sample);
    target.push_back(transformed);
  }

  const AlignmentResult result = estimatePlanarAlignment(
      source, target, 60U, 10.0, 0.20, 400, 5U);
  EXPECT_NEAR(wrapAngle(result.transform.yaw - expected.yaw), 0.0,
              1.0e-8);
  EXPECT_NEAR(result.transform.tx, expected.tx, 1.0e-5);
  EXPECT_NEAR(result.transform.ty, expected.ty, 1.0e-5);
  EXPECT_NEAR(result.transform.tz, expected.tz, 1.0e-5);
  EXPECT_GT(std::count(result.inlier_mask.begin(),
                       result.inlier_mask.end(),
                       static_cast<std::uint8_t>(1U)),
            105);
  EXPECT_LT(result.horizontal_rmse_m, 1.0e-7);
}

TEST(TrajectoryAlignment, ComputesAbsoluteAndRelativeErrors) {
  const PlanarTransform transform{-12.0 * kPi / 180.0, 300.0, 500.0,
                                  2.0};
  std::vector<PoseSample> estimates;
  std::vector<PoseSample> references;
  for (int index = 0; index < 100; ++index) {
    PoseSample estimate;
    estimate.stamp = index * 0.1;
    estimate.x = 0.2 * index;
    estimate.y = std::sin(0.05 * index);
    estimate.z = 0.0;
    estimate.yaw = 0.05 * std::sin(0.02 * index);
    estimates.push_back(estimate);
    references.push_back(transform.apply(estimate));
  }
  const std::vector<MatchedPose> matches =
      associateTrajectories(estimates, references, 0.01, 0.2);
  const std::vector<ErrorRow> rows = buildErrorRows(matches, transform);
  ASSERT_EQ(rows.size(), estimates.size());
  std::vector<double> horizontal_errors;
  for (const ErrorRow& row : rows) {
    horizontal_errors.push_back(row.horizontal_error_m);
  }
  EXPECT_LT(calculateDistribution(horizontal_errors).rmse, 1.0e-9);

  const RpeResult rpe = computeRpe(matches, transform, 1.0);
  EXPECT_GT(rpe.translation_error_m.count, 50U);
  EXPECT_LT(rpe.translation_error_m.rmse, 1.0e-9);
  EXPECT_LT(rpe.yaw_error_deg.rmse, 1.0e-9);
}

}  // namespace
}  // namespace onion_gps_evaluation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
