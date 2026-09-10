#include <gtest/gtest.h>

#include "localization_ekf.hpp"

namespace localization_pkg {
namespace {

GpsSample make_gps(double stamp_sec, double x_m, double y_m, double z_m = 0.0) {
  GpsSample sample;
  sample.stamp_sec = stamp_sec;
  sample.position_m = Eigen::Vector3d(x_m, y_m, z_m);
  sample.covariance_m2 = Eigen::Matrix3d::Identity() * 0.25;
  return sample;
}

ImuSample make_stationary_imu(double stamp_sec) {
  ImuSample sample;
  sample.stamp_sec = stamp_sec;
  sample.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.80665);
  return sample;
}

TEST(LocalizationEkfTest, InitializesYawFromGpsTravelDirection) {
  FilterConfig config;
  config.minimum_yaw_initialization_distance_m = 1.0;
  LocalizationEkf filter(config);
  EXPECT_EQ(filter.correct_gps(make_gps(1.0, 0.0, 0.0)), UpdateResult::kWaitingForInitialization);
  EXPECT_EQ(filter.correct_gps(make_gps(2.0, 0.0, 2.0)), UpdateResult::kAccepted);
  EXPECT_TRUE(filter.is_initialized());
  const Eigen::Vector3d forward = filter.orientation() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(forward.x(), 0.0, 1e-6);
  EXPECT_NEAR(forward.y(), 1.0, 1e-6);
}

TEST(LocalizationEkfTest, StationaryImuPreservesPositionAndUnitQuaternion) {
  LocalizationEkf filter(FilterConfig{});
  filter.correct_gps(make_gps(1.0, 0.0, 0.0));
  filter.correct_gps(make_gps(2.0, 2.0, 0.0));
  ASSERT_TRUE(filter.is_initialized());
  ASSERT_TRUE(filter.propagate(make_stationary_imu(2.1)));
  ASSERT_TRUE(filter.propagate(make_stationary_imu(2.2)));
  EXPECT_NEAR(filter.position_m().norm(), 2.0, 1e-5);
  EXPECT_NEAR(filter.orientation().norm(), 1.0, 1e-12);
}

TEST(LocalizationEkfTest, RejectsOutOfOrderImuAndGpsOutlier) {
  LocalizationEkf filter(FilterConfig{});
  filter.correct_gps(make_gps(1.0, 0.0, 0.0));
  filter.correct_gps(make_gps(2.0, 2.0, 0.0));
  ASSERT_TRUE(filter.propagate(make_stationary_imu(2.1)));
  EXPECT_FALSE(filter.propagate(make_stationary_imu(2.0)));
  EXPECT_EQ(filter.correct_gps(make_gps(2.2, 1000.0, 1000.0)), UpdateResult::kRejected);
}

TEST(LocalizationEkfTest, GpsCorrectionReducesPositionError) {
  LocalizationEkf filter(FilterConfig{});
  filter.correct_gps(make_gps(1.0, 0.0, 0.0));
  filter.correct_gps(make_gps(2.0, 2.0, 0.0));
  ASSERT_TRUE(filter.propagate(make_stationary_imu(2.1)));
  const double before_error_m = (filter.position_m() - Eigen::Vector3d(2.2, 0.0, 0.0)).norm();
  EXPECT_EQ(filter.correct_gps(make_gps(2.2, 2.2, 0.0)), UpdateResult::kAccepted);
  const double after_error_m = (filter.position_m() - Eigen::Vector3d(2.2, 0.0, 0.0)).norm();
  EXPECT_LT(after_error_m, before_error_m);
}

}  // namespace
}  // namespace localization_pkg

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
