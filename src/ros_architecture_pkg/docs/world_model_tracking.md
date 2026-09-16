# LiDAR map tracking 개발 계약

## 목적

첫 World Model vertical slice는 LiDAR scan-local AABB를 측정시각의 `map` 좌표로
변환하고 짧은 시간 동안 persistent ID로 추적한다. 이는 RViz가 서로 다른 토픽을
겹쳐 그리는 것과 달리 downstream이 읽을 수 있는 명시적 map-frame 데이터다.

## 좌표와 시각

- 입력 중심과 크기는 `LidarObservationArray.header.frame_id=lidar_link` 기준이다.
- 변환은 반드시 원본 scan stamp의 `map <- lidar_link` TF를 사용한다.
- latest TF 또는 callback 완료시각으로 과거 관측을 현재 위치에 붙이지 않는다.
- 출력 `WorldModel.header.stamp`는 accepted scan stamp인 fusion reference time이다.
- `TrackedObject.source_stamp`와 provenance/calibration ID는 마지막 기여 관측을 보존한다.

입력 AABB에는 객체 yaw가 없으므로 map 회전 후 여덟 꼭짓점을 포함하는 map-aligned
AABB로 보수화한다. identity orientation은 객체 heading 추정이 아니라 AABB 축 의미다.

## 추적과 수명

map XY 거리 gate 안에서 deterministic nearest-neighbour association을 수행한다.
scan-local ID는 association tie나 persistent identity로 사용하지 않는다. 새 객체는
tentative, 충분히 재관측되면 confirmed, 일시 누락은 coasting이다. coasting 상한을
넘은 track은 삭제하며 과거 장애물을 무기한 유지하지 않는다. Localization reset,
clock reset과 관측 stamp 역행은 모든 temporal state를 무효화한다.

## 안전 경계

현재 입력은 개발 calibration/freshness가 미검증이고 검출·Localization·TF 오차를
합친 위치 불확실성 상한도 없다. 따라서 이 구현은 `objects_valid=true`인 map geometry를
보여줄 수 있지만 `objects_verified=false`, `planner_ready=false`다. 주행 활성화에는
다음 검증과 같은 계약 변경이 별도로 필요하다.

- 실제 MORAI loadout에서 LiDAR extrinsic과 축 검증
- scan period·jitter·processing/TF latency의 p95/p99 측정과 timeout 승인
- 객체 위치·크기 오차 및 Localization covariance를 합친 inflation budget
- 합류·가림·분할·병합 상황의 association 및 velocity 오차 검증
- HD Map/Route/free-space 계층 구현과 producer-consumer planner 테스트
