# 기존 LiDAR 검출부 이식 계약

[설계] 기존 ROI → VoxelGrid → DBSCAN → 축 정렬 bounding box를 이식한다.
공개 토픽·노드·frame은 변경하지 않는다. `messages/lidar_messages.yaml`이
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
