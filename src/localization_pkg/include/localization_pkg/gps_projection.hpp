/*
gps_projection.hpp
- 역할: WGS84 GPS 좌표를 대회 map 좌표로 투영한다.
- 주요 클래스: GpsProjection
- ROS 인터페이스: 없음
*/
#pragma once

#include <string>

namespace localization_pkg {

struct MapPoint {
  // GPS를 투영해 얻은 map frame 기준 3차원 위치를 m 단위로 보관한다.
  // 현재 localization은 2D로 동작하므로 z는 설정된 map 높이를 사용한다.
  double x;
  double y;
  double z;
};

class GpsProjection {
  // 입력: WGS84 위도·경도와 생성 시 지정한 UTM zone 및 map 원점.
  // 처리: GeographicLib으로 UTM 좌표를 구하고 공식 원점을 빼 map 좌표로 변환한다.
 // 출력: 성공 시 MapPoint, 실패 시 false와 선택적인 오류 문자열을 반환한다.
 // 책임 경계: GPS 상태, timestamp 및 covariance 검증은 담당하지 않는다.
 public:
  // 함수이름: GpsProjection
  // 기능: UTM 영역, 반구, map 원점과 출력 Z 좌표를 저장한다.
  // 인자: utm_zone, north_hemisphere, origin_easting_m, origin_northing_m, map_z
  // 반환값: 없음
  GpsProjection(int utm_zone, bool north_hemisphere, double origin_easting_m,
                double origin_northing_m, double map_z);

  // 함수이름: project
  // 기능: WGS84 좌표를 설정된 UTM 영역 기반 map 좌표로 투영한다.
  // 인자: latitude_deg, longitude_deg, altitude_m, point, error
  // 반환값: 투영에 성공하면 true, 입력이나 설정이 잘못되면 false
  bool project(double latitude_deg, double longitude_deg, double altitude_m,
               MapPoint* point, std::string* error) const;

 private:
  int utm_zone_;
  bool north_hemisphere_;
  double origin_easting_m_;
  double origin_northing_m_;
  double map_z_;
};

}  // namespace localization_pkg

