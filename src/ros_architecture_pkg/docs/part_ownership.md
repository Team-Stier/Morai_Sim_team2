# 파트 배분 및 소유권

사람 이름이 정해지기 전의 역할 기반 배분이다. 한 사람이 여러 파트를 맡을 수 있지만 패키지 책임과 승인 경계는 합치지 않는다.

| 파트 | 소유 패키지 | 주 책임 | 완료 전 선행조건 |
|---|---|---|---|
| System Architecture & Integration | `ros_architecture_pkg`, `common_msgs_pkg`, `system_bringup_pkg` | 중앙 계약, 공유 타입, 전체 launch, 의존성·통합 승인 | 공식 UDP 명세와 각 파트 논리 I/O |
| Simulator Platform | `morai_interface_pkg` | 허용 UDP, 패킷 검증, 정규화, 연결 health | 실제 대회 패킷 및 LAN 검증 |
| HD Map & Route | `hd_map_pkg`, `global_route_manager_pkg` | 정적 지도, topology, 전역경로, 체크포인트 진행도 | 공식 지도 원본·맵명·좌표계 확인 |
| Localization | `localization_pkg` | ego pose·velocity·quality, GPS blackout 연속 추정 | sensor extrinsic/time 기준, map layer |
| Camera Perception | `camera_perception_pkg` | 객체·차선·신호·주행 가능 영역 관측 | 고정 카메라 calibration과 수신 검증 |
| LiDAR Perception | `lidar_perception_pkg` | 지면·3D 객체·장애물·free-space 관측 | VLP16 설정과 calibration 검증 |
| World Model | `world_model_pkg` | 시간 정렬, 좌표 변환, cross-sensor fusion, tracking | Localization pose history와 모든 관측 계약 |
| Planning | `path_planning_pkg` | behavior와 시간 파라미터 local trajectory | world model, route context, 차량 제약 |
| Control & Safety | `vehicle_control_pkg`, `safety_supervisor_pkg` | trajectory tracking, final gate, watchdog, safe stop | 제어 패킷 의미와 차량 응답 검증 |
| Evaluation & QA | `runtime_evaluation_pkg` | 규정 지표, 지연, 위반, 예상 패널티 기록 | 관측 전용 계약과 기준 시계 |

## 승인 권한

- 파트 담당자는 자기 패키지 내부 알고리즘과 로컬 파라미터를 소유한다.
- 공개 인터페이스 변경은 System Architecture & Integration 검토 없이 병합하지 않는다.
- MORAI 외부 통신 변경은 Simulator Platform과 System Architecture가 함께 승인한다.
- 최종 제어 경로 변경은 Control & Safety와 System Architecture가 함께 승인한다.
- HD Map 기준·좌표계 변경은 HD Map & Route, Localization, World Model이 함께 검증한다.
- 센서별 calibration artifact는 해당 Perception/Localization 파트가 소유하고, frame·방향·단위와 활성 version은 System Architecture가 승인한다.

## 통합 순서

1. 중앙 논리 계약 및 패킷 증거
2. MORAI interface와 공유 타입
3. HD Map/Route 및 Localization 기준
4. Camera/LiDAR 독립 관측
5. World Model 시간·좌표 융합
6. Planning과 Controller open-loop
7. Safety final gate
8. Full bringup, runtime evaluation과 MORAI closed-loop
