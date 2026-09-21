# Closedteam2 제어 통합 계약 변경안 및 적용 범위

사용자의 Closedteam2 컨트롤 이식 요청에 따라 기존 공개 이름을 유지하고,
예약된 Trajectory와 ControllerStatus 스키마를 아래 중앙 모듈로 구체화한다.

- Producer: 향후 path_planner_node가 odom trajectory와 planning status를 발행한다.
  Trajectory는 Localization reset_id와 planning reference stamp를 보존한다.
- Consumer: vehicle_controller_node는 승인된 Odometry/LocalizationStatus와
  Trajectory/ComponentStatus 네 입력만 읽는다. 결과는 nominal ActuatorCommand다.
- Safety, Readiness, Runtime Evaluation의 예약 consumer는 이 스키마를 사용한다.
  Safety와 Planner 구현 및 MORAI UDP 활성화는 이 변경에 포함되지 않는다.
- 기존 ActuatorCommand wire layout은 유지한다. nominal.valid는 추종 계산의
  유효성만 의미하며 Safety 승인이나 실제 차량 구동 권한이 아니다.
- 상세 필드: config/messages/controller_messages.yaml.
  기존 중앙 rate/queue를 사용하며 timeout은 현재 계약의 미확정 상태를 유지한다.

Closedteam2의 조향 코어(Pure Pursuit, Stanley, HybridSupervisor)와
BoundedPiController의 속도 오차 제어를 vehicle_control_pkg 내부로 이식한다.
원본 global/vision/avoidance 토픽, speed policy, mission planning, 안전 명령 노드와
UDP adapter는 실행하지 않는다. 목표 속도는 Planner trajectory에서만 받는다.
GPS blackout 단독으로 정지하지 않으며 local odometry의 validity와 품질을 따른다.
현재 Localization 개발 모드의 stop_required=true는 그대로 존중한다.

Trajectory는 전진용 ordered local path다. poses/speed_mps/time_from_start는
같은 길이이며 시간은 0부터 엄격히 증가한다. 유효기간은 마지막 샘플 시간 이하다.
각 quaternion은 단위 회전이어야 하며 속도는 음수일 수 없다. 정지 요청은
stop_required=true와 모든 목표 속도 0으로 표현하며 빈 경로도 허용한다.
Planner는 경로 분기 선택, 장애물 처리와 곡률/속도/감속 가능성을 소유한다.
Controller는 현재 reference time의 속도를 선형 보간해 추종한다.

사용자의 추가 지시에 따라 별도 NaN/범위/freshness 검사, watchdog 또는
추가 saturation/rate-limit 계층은 추가하지 않는다. 원본 조향 코어와 PI에
이미 있는 검사·제한은 그대로 유지한다. 입력 수신 여부 확인과 기존
valid/ready/stop_required를 따르는 처리는 ROS 계약 연결의 일부다.
출력 command는 생성시각, status는 평가시각을 사용한다. 입력 측정 stamp는
바꾸지 않고 status에 남긴다. 소비자용 스키마의 조건을 Producer가 만족해야 한다.

검증: 원본 조향 단위시험, 원본 PI, trajectory/단위 변환, ROS callback 연결,
중앙 계약과 빌드. 실제 MORAI closed-loop와 조향 부호/스케일 튜닝은 미검증이다.
