# 기존 LiDAR 검출부 이식 계약

[설계] 기존 ROI → VoxelGrid → DBSCAN → 축 정렬 bounding box를 이식한다.
공개 출력 토픽·노드·frame은 변경하지 않는다. `messages/lidar_messages.yaml`이
새 메시지 필드의 단일 원본이다. 생산자는 `lidar_perception_node`, 소비자는
예약된 `world_model_node`이며 소비자 런타임 구현은 이번 범위가 아니다.

`objects_valid`는 입력 스캔 처리가 성공했다는 뜻이다. 빈 목록은 검출 0개이며
관측 영역 전체가 비어 있다는 뜻이 아니다. 입력 실패는 빈 결과와
`objects_valid=false`로 전달한다. 시각이 유효하지 않은 입력은 관측을 발행하지
않고 ComponentStatus로만 알린다. 결과에 과거 객체를 재사용하지 않는다.

객체 중심·크기는 lidar_link의 m 단위이며 ID는 해당 스캔에서만 유효하다.
confidence=-1은 확률 보정 미실시다. 속도·지면·free-space·occupancy는 이번
검출기에 구현하지 않으며 해당 기능을 추정했다고 표시하지 않는다.

[미확정] 현재 센서 축·활성 loadout·장착 위치 검증이 끝나지 않았다.
calibration_id는 중앙에서 사용자 승인한 2026-09-11 장착 위치 `(2, 0, 1.5)`와
영 회전을 식별한다. 실측 검증 인증이 아니다.
calibration_verified=false, freshness_verified=false인 관측은 개발 진단용이다.
World Model은 이 관측으로 planner-ready scene을 만들면 안 된다.
ComponentStatus는 DEGRADED, ready=false, stop_required=true를 유지한다.
현재 입력의 ingress_fallback 시각을 보존하며 TF는 발행하지 않는다.

## Roll/pitch 수평화 입력 확장 (2026-09-14)

사용자가 요청한 `horizontal_pkg/Paik` 로직 이식을 위해 DBSCAN producer의
입력에 `/molit/localization/ego_state`를 추가한다. 구현 소유자는
`lidar_perception_pkg`이며 용도는 스캔 시각의 roll/pitch에 의한 센서 로컬
전처리뿐이다. World Model이 소유한 전역 좌표 객체 융합·추적·센서 동기화는
수행하지 않는다. localization producer의 wire 형식과 주기는 그대로다.

스캔 시각을 양쪽에서 감싸는 유효 자세를 SLERP하며 시간 외삽은 금지한다.
`messages/lidar_runtime.yaml#horizontalization`의 개발 시간·큐 제한을 적용하고,
pose_valid의 roll/pitch/yaw, frame, quaternion, 단조 시각, reset_id를 검사한다.
초기화 경계·잘못된 자세에서는 이력을 비우고 대기 중 스캔을 무효화한다.
자세 단절·누락·과도한 간격에서 원본 점군으로 조용히 대체하지 않는다.

중앙 static `base_link → lidar_link`의 회전을 조회하여 장착각을 포함한다.
수평화의 원점은 LiDAR이고 yaw는 유지한다. ROI 수치는 이 임시 수평 좌표에서
적용한다. `z_min`은 센서 원점 기준 수평 높이이며 도로 기준 높이가 아니다.
하한과 상한은 포함한다. 보정 후에도 ROI 안에 있는 지면은 자동으로 제거되지 않는다.
이후 박스 8개 모서리를 역회전한 enclosing AABB와 voxel 점군을
`lidar_link`로 반환한다. 기존 public 관측/consumer·ROS1 MD5·측정 stamp는
변하지 않는다. 수평화용 새 TF/frame/공개 점군 토픽은 발행하지 않는다. 이 변환은
거리 노이즈 제거·지면 추정·scan 내 motion deskew가 아니다.

기존 미보정 비교는 `leveling_enabled:=false`로 명시적으로 선택한다.
사전학습 대체 backend에는 이 전처리를 적용하지 않는다. 개발 validity와
주행 readiness는 구별하며 ready=false, stop_required=true를 유지한다.

## 원본 군집 점 표시 확장 (2026-09-14)

사용자의 HD Map 위 원본 장애물 점 표시 요청에 따라 공개 표시용
`/molit/perception/lidar/cluster_points` (`sensor_msgs/PointCloud2`)를 승인한다.
producer는 `lidar_perception_node`, consumer는 `vehicle_visualizer_node`다.
World Model·Planner 입력이나 지면/전체 occupancy 출력으로 사용하지 않는다.

채택된 DBSCAN voxel에 속한 모든 ROI 내 raw point record를 원본 순서로 복사한다.
원래 필드 바이트와 `lidar_link`, 원 scan stamp를 유지하고 UINT32 `cluster_id`
(같은 스캔의 observation scan_local_id), `source_index`(원본 row-major 인덱스)를
추가한다. ROI는 수평 좌표로 판단하지만 발행 XYZ는 역회전 계산값도 아닌 원본
바이트다. noise·ROI 밖·거절된 군집의 점은 포함하지 않는다. 출력은 height=1의
비조직 점군이며 원본 record padding은 보존, 원본 row padding은 제거한다.

Voxel은 PCL과 같은 global cell 좌표와 x-fastest 정렬을 사용하되 sparse map에
원본 인덱스를 보존한다. centroid 계산은 double 누적 후 float으로 변환하므로
기존 PCL centroid와 극소수 수치 차이는 가능하다. dense leaf layout은 사용하지
않는다. 대표 점군의 PCL 결과와 허용 오차 내 일치를 검사한다.

잘못된 입력이나 정렬 실패는 빈 점군으로 표시를 지우고, 입력 단절은 consumer의
ROS/wall timeout으로 삭제한다. 시각화는 기존 private MarkerArray에서 POINTS를
기본으로 사용하며 scan-time TF만 허용한다. XYZ 높이를 HD Map 평면에 맞춰
덮어쓰지 않는다. Localization reset 이전 스캔도 새 표시로 재사용하지 않는다.
기존 observation의 wire 형식·박스 데이터는 유지하며 RViz 장애물 표시는 점이다.

개발 중 입력 수신 감시는 기존 bridge watchdog의 1초·2Hz 계약을 사용한다.
이는 검출 결과의 승인된 주행 freshness 임계값이 아니다. 별도 max_scan_age_sec는
기본 0(미확정)이며 수신 timeout으로 대체하지 않는다. 개별 점군의 age,
clock 역행/미래/중복, 수신 단절을 검사한다. 주행 readiness 활성화는 실제
p95/p99 지연과 축 검증 후 중앙 계약을 갱신하는 별도 작업이다.

ComponentStatus는 status_period_sec=0.5에 따라 타이머에서만 발행한다.
scan과 transport 콜백은 상태를 갱신하며 오류도 다음 heartbeat에서 전달한다.

2026-09-11 시각화 확장: `vehicle_visualizer_node`가 같은 관측을 읽고
scan-time TF로 RViz 박스를 표시한다. World Model 융합 허용 여부는 그대로 유지한다.

## 사전학습 분류 확장 (2026-09-14, messages v0.2.0)

사용자가 요청한 사전학습 모델 연결을 위해 객체 메시지에 `semantic_class`,
`learned_box`, `model_class`, `model_score`를 추가한다. producer는
`lidar_perception_node`, 현재 consumer는 `vehicle_visualizer_node`와 공통
검증기다. 예약 consumer World Model도 새 메시지로 빌드해야 한다.
ROS1 MD5가 변경되므로 producer/consumer를 함께 재시작한다. 토픽 이름·
frame·단위·원 측정 stamp·개발 안전 gate는 유지한다.

- `semantic_class`: UNKNOWN=0, PEDESTRIAN=1, VEHICLE=2, OTHER=3.
  pedestrian은 나이·성별 의미가 없다. 차량은 car/truck/construction_vehicle/
  bus/trailer이며, motorcycle/bicycle/barrier/traffic_cone는 OTHER다.
- `learned_box=false`: 기존 DBSCAN이며 UNKNOWN, 빈 model_class,
  model_score=0. point_count는 ROI/VoxelGrid 이후 cluster 점 개수다.
- `learned_box=true`: 공식 nuScenes PointPillars-MultiHead의 예측 OBB를
  완전히 포함하는 lidar_link AABB다. point_count는 OBB 내부 유한 raw XYZI
  점 개수(>0)다. 회전 차량의 AABB size를 차체 실측 크기로 해석하지 않는다.
- model_class는 위 10개 원 모델 클래스 중 하나이며 semantic_class와
  일치해야 한다. model_score는 [0,1]의 보정되지 않은 모델 점수다.
  confidence는 -1을 유지하며 점수와 혼용하지 않는다.
- 지면·free-space·occupancy·velocity capability는 계속 false다.
  모델 실패는 invalid empty observation 또는 시각/epoch 무효 시 status만
  발행한다. 정상 검출 0개는 free-space 증명이 아니다.
- standalone 대체 executable `learned_lidar_node.py`는 기존 node와 상호
  배타적으로 실행한다. 모델 입력 전처리·임계값은 패키지 로컬 설정에 둔다.

현재 consumer는 semantic fields를 검증한 뒤 진단 색상/라벨에만 사용한다.
World Model의 for_fusion 검증은 개발 관측을 계속 거부한다. 추가 학습,
VLP16 정확도 평가와 주행 readiness 승인은 이번 확장에서 하지 않는다.
