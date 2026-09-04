#include <gtest/gtest.h>

#include <limits>

#include <GeographicLib/UTMUPS.hpp>

#include "localization_pkg/gps_projection.hpp"

namespace localization_pkg {
namespace {

constexpr int kUtmZone = 52;
constexpr bool kNorthHemisphere = true;
constexpr double kOriginEastingM = 302595.0;
constexpr double kOriginNorthingM = 4124145.0;
constexpr double kMapZ = 0.0;

// 공식 UTM zone 52 North와 map 원점을 사용하는 GPS projector를 만든다.
GpsProjection makeProjector() {
  return GpsProjection(kUtmZone, kNorthHemisphere, kOriginEastingM,
                       kOriginNorthingM, kMapZ);
}

// 아래 TEST들은 공식 좌표의 투영 결과와 WGS84 범위, UTM zone·반구,
// non-finite 입력 및 null 출력 포인터의 거부 경로를 검증한다.
TEST(GpsProjectionTest, ProjectsWgs84FixRelativeToOfficialOrigin) {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  GeographicLib::UTMUPS::Reverse(kUtmZone, kNorthHemisphere,
                                 kOriginEastingM + 10.0,
                                 kOriginNorthingM - 5.0, latitude_deg,
                                 longitude_deg);

  MapPoint point;
  std::string error;
  const GpsProjection projector = makeProjector();

  ASSERT_TRUE(projector.project(latitude_deg, longitude_deg, 123.0, &point,
                                &error));
  EXPECT_NEAR(point.x, 10.0, 1e-3);
  EXPECT_NEAR(point.y, -5.0, 1e-3);
  EXPECT_DOUBLE_EQ(point.z, kMapZ);
  EXPECT_TRUE(error.empty());
}

TEST(GpsProjectionTest, RejectsInvalidLatitude) {
  MapPoint point;
  std::string error;
  const GpsProjection projector = makeProjector();

  EXPECT_FALSE(projector.project(91.0, 127.0, 0.0, &point, &error));
  EXPECT_FALSE(error.empty());
}

TEST(GpsProjectionTest, RejectsFixOutsideConfiguredUtmZone) {
  MapPoint point;
  std::string error;
  const GpsProjection projector = makeProjector();

  EXPECT_FALSE(projector.project(37.0, 121.0, 0.0, &point, &error));
  EXPECT_FALSE(error.empty());
}

TEST(GpsProjectionTest, RejectsFixOutsideConfiguredHemisphere) {
  MapPoint point;
  std::string error;
  const GpsProjection projector = makeProjector();

  EXPECT_FALSE(projector.project(-37.0, 127.0, 0.0, &point, &error));
  EXPECT_FALSE(error.empty());
}

TEST(GpsProjectionTest, RejectsNonFiniteCoordinates) {
  MapPoint point;
  std::string error;
  const GpsProjection projector = makeProjector();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(projector.project(nan, 127.0, 0.0, &point, &error));
  EXPECT_FALSE(projector.project(37.0, nan, 0.0, &point, &error));
}

TEST(GpsProjectionTest, RejectsNullOutputPoint) {
  std::string error;
  const GpsProjection projector = makeProjector();

  EXPECT_FALSE(projector.project(37.0, 127.0, 0.0, nullptr, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace localization_pkg

int main(int argc, char** argv) {
  // 이 파일의 GoogleTest 항목을 등록하고 전체 결과를 반환한다.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

