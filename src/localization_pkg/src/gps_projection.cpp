/*
gps_projection.cpp
- 역할: GeographicLib을 사용한 WGS84 GPS 좌표의 map 좌표 투영을 구현한다.
- 주요 클래스: GpsProjection
- ROS 인터페이스: 없음
*/
#include "localization_pkg/gps_projection.hpp"

#include <cmath>

#include <GeographicLib/UTMUPS.hpp>

namespace localization_pkg {
namespace {

// 함수이름: setError
// 기능: 호출자가 오류 출력을 제공한 경우에만 실패 이유를 기록한다.
// 인자: error - 출력 문자열 포인터, message - 기록할 내용
// 반환값: 없음
void setError(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

}  // namespace

// 함수이름: GpsProjection
// 기능: 모든 GPS fix에 적용할 투영 기준값을 저장한다.
// 인자: utm_zone, north_hemisphere, origin_easting_m, origin_northing_m, map_z
// 반환값: 없음
GpsProjection::GpsProjection(int utm_zone, bool north_hemisphere,
                             double origin_easting_m,
                             double origin_northing_m, double map_z)
    : utm_zone_(utm_zone),
      north_hemisphere_(north_hemisphere),
      origin_easting_m_(origin_easting_m),
      origin_northing_m_(origin_northing_m),
      map_z_(map_z) {}

// 함수이름: project
// 기능: WGS84 좌표를 UTM으로 투영하고 설정된 원점을 빼 map 좌표를 만든다.
// 인자: latitude_deg, longitude_deg, altitude_m, point, error
// 반환값: 투영에 성공하면 true, 입력이나 설정이 잘못되면 false
bool GpsProjection::project(double latitude_deg, double longitude_deg,
                            double /* altitude_m */, MapPoint* point,
                            std::string* error) const {
  if (error != nullptr) {
    error->clear();
  }
  if (point == nullptr) {
    setError(error, "map point output is null");
    return false;
  }
  if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
      latitude_deg < -90.0 || latitude_deg > 90.0 || longitude_deg < -180.0 ||
      longitude_deg > 180.0) {
    setError(error, "latitude or longitude is outside WGS84 bounds");
    return false;
  }
  if (utm_zone_ < GeographicLib::UTMUPS::MINUTMZONE ||
      utm_zone_ > GeographicLib::UTMUPS::MAXUTMZONE ||
      !std::isfinite(origin_easting_m_) || !std::isfinite(origin_northing_m_) ||
      !std::isfinite(map_z_)) {
    setError(error, "projection configuration is invalid");
    return false;
  }

  int zone = GeographicLib::UTMUPS::INVALID;
  bool north_hemisphere = false;
  double easting_m = 0.0;
  double northing_m = 0.0;
  try {
    GeographicLib::UTMUPS::Forward(latitude_deg, longitude_deg, zone,
                                   north_hemisphere, easting_m, northing_m);
  } catch (const GeographicLib::GeographicErr& exception) {
    setError(error, exception.what());
    return false;
  }

  if (zone != utm_zone_ || north_hemisphere != north_hemisphere_) {
    setError(error, "GPS fix is outside the configured UTM zone or hemisphere");
    return false;
  }
  const double map_x = easting_m - origin_easting_m_;
  const double map_y = northing_m - origin_northing_m_;
  if (!std::isfinite(easting_m) || !std::isfinite(northing_m) ||
      !std::isfinite(map_x) || !std::isfinite(map_y)) {
    setError(error, "projected map point is non-finite");
    return false;
  }

  *point = MapPoint{map_x, map_y, map_z_};
  return true;
}

}  // namespace localization_pkg

