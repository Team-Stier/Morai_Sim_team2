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
calibration_id는 저장 프로필 SHA256 참조이며 검증 인증이 아니다.
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
