# 터널 벽 정합 연결 — 2026-09-23

## 데이터와 소유권

HD Map은 원본 MGeo wall 객체 두 개의 열린 XYZ 벽선을 중앙 승인
`/molit/map/static_walls` (`common_msgs_pkg/StaticWallMap`)로 latched 발행한다.
새 메시지의 source hash는 원본 object_set.json, map_id는 선택 벽선과 모델
불확실성을 식별한다. 표시용 벽 높이는 메시지에 없으며 정합에 사용하지 않는다.
기존 HdMap wire는 바뀌지 않아 planner/world model의 메시지 호환성을 유지한다.

Localization만 raw LiDAR를 지도에 정합한다. 카메라/인식 내부 토픽, 전역경로,
체크포인트, 현재 차량이 중앙선이라는 가정, 숨은 시뮬레이터 상태는 관측에 쓰지 않는다.

## 측정과 추정

- 중앙 LiDAR mount [2,0,1.5]m와 측정시각의 보간 IMU로 점군을 회전·이동한다.
- 중앙 60ms reorder queue에서 원본 LiDAR stamp로 처리한다. 미래/역행/중복/지연,
  bracketing IMU 부재, 잘못된 frame, 미초기화 필터는 거부한다.
- 높은 벽점만 선택하고 양쪽 벽의 최소 점 개수, 종방향 관측 길이, 방향 차이,
  간격 오차와 잔차를 검사한다. 한쪽 벽만으로 업데이트하지 않는다.
- 원본 벽의 지배적인 법선 방향으로 위치·속도·가속도 bias의 Kalman gain을 제한한다.
  Joseph covariance update로 PSD를 유지한다. 터널 진행방향의 상태와 분산은
  이 보정으로 줄이지 않는다. 자세는 기존 IMU 관측을 유지한다.
- map 위치만 측정 보정한다. odom 위치는 기존처럼 예측 이동량을 적분하며
  wall update 자체로 점프하지 않는다. 다음 원본 IMU 시각에 pose/status/TF를 발행한다.
- 유효 GPS 초기화가 먼저 필요하다. 평행 벽은 종방향 위치를 결정하지 못하므로
  LOST 상태에 임의 위치를 주입하거나 벽만으로 재초기화하지 않는다.

GPS 미수신 15초 이후의 개발 유효성은 신선한 벽 정합(0.5초), GPS 기준 최대
60초, 수평 위치 표준편차 3m 이하를 모두 만족할 때만 연장한다. IMU/estimate
freshness와 `stop_required=true`는 유지한다. 종방향 분산 증가로 조건이 깨지면
LOST가 맞다. 이는 무제한 터널 주행이나 독립 LiDAR odometry 구현이 아니다.

## 검증과 실제 한계

합성 벽/점군으로 횡오차 감소, 진행방향 공분산 보존(상관 prior 포함), 한쪽 벽,
짧은 장애물, 폭 불일치, 방향 불일치, NaN, 지연, 초기화 부재를 검사했다.
별도 ROS master에서 실제 StaticWallMap/PointCloud2 producer→Localization
consumer와 GPS blackout 상태 전이를 검증했다. 기존 7개 ROS 추정기 테스트도 통과했다.

현장 저장 LiDAR는 여러 가정 종방향 위치에서 약 0.16m RMS로 벽과 맞았다.
서로 다른 종방향 위치에서 모두 맞는 것은 절대 위치 검증이 아니라 퇴화의 증거다.
임의 seed는 오프라인 호환성 확인에만 사용했고 runtime에 주입하지 않았다.

실시간 재진입에서 `wall matched lateral=-0.003m rms=0.148m points=[343,329]`
진단을 확인했다. 당시 GPS 경과 46.5초, 전체 수평 표준편차 64.7m로 LOST였다.
진행방향 drift가 커지면 유한 벽선 범위를 벗어나 정합도 거부된다. 따라서
벽 정합 연결 및 횡보정은 확인했지만 터널 전체 위치 유지/closed-loop 성공은
확인하지 못했다. 검증된 차속 또는 퇴화 검사를 갖춘 추가 LiDAR odometry 등
진행방향 관측이 후속으로 필요하다.

중앙 아키텍처 테스트의 기존 saved sensor profile SHA 불일치는 외부 파일 변경이다.
현재 GPS/IMU/LiDAR mount와 회전은 중앙 설정과 일치함을 별도로 읽어 확인했으며,
저장 프로필의 기존 provenance 해시를 임의로 갱신하지 않았다.

재진입 180초 기록에는 유효 상태 1,714개, 벽 정합 진단과 유효 상태가 함께
나온 메시지 1,461개가 있었다(메시지 개수이며 독립 정합 횟수가 아니다).
최장 유효 GPS 경과는 14.982초였다. 그때 이미 수평 표준편차가 9.424m여서
3m 조건을 만족하지 못했고, 벽 보조 60초 연장은 이 실주행에서 활성화되지 않았다.
따라서 **기존 터널 15초 이후 끊김은 이번 횡보정만으로 해결되지 않았다.**
[상태 집계](evidence/tunnel_wall_live_20260923.json)를 참고한다.

검증 결과: 빌드 성공, 새 벽 매칭/유효성 단위 테스트 7개와 공유 계약/직렬화
테스트 2개, 기존 GPS/IMU 단위 테스트 17개, ROS 통합 테스트 8개 통과.
HD Map 56개, 공유 메시지 전체 42개, Localization 계약 2개, 지도 표시 5개,
중앙 공개 계약 29개 통과. TF/timestamp 17개 중 16개 통과, 외부 저장 프로필
SHA 검사 1개는 원래 작업 브랜치에서도 같은 실패가 재현됐다.
중앙 다이어그램 생성·SVG/PNG manifest 검사도 통과했다.
