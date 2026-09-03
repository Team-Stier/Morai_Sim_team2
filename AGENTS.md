# Morai_Sim_team2 AI 작업 계약

이 파일은 저장소 전체에 적용된다. 사람과 AI 모두 아래 순서와 경계를 따라야 한다.

## 작업 전 필수 읽기

1. 루트 `README.md`
2. `src/ros_architecture_pkg/README.md`
3. `src/ros_architecture_pkg/config/interface_contract.yaml`
4. TF 작업은 `src/ros_architecture_pkg/config/tf/`, 시간 작업은 `config/timestamp/`, MORAI 통신 작업은 `config/morai_interface/`
5. 작업 대상 패키지의 `README.md`
6. 관련 공식 규정과 `참고파일들/` 원본

## 중앙 ROS 계약 잠금

- `ros_architecture_pkg`는 공개 ROS node, topic, message, service, action, TF frame, 단위, timestamp, 주기, queue, timeout, 상태와 소유 패키지를 정하는 유일한 권위다.
- `config/interface_contract.yaml`이 중앙 진입점이며 그 파일이 가리키는 `config/tf/`와 `config/timestamp/`도 같은 중앙 계약의 일부다.
- 다른 패키지는 공개 인터페이스의 이름이나 의미를 독자적으로 정의하거나 변경하지 않는다.
- 중앙 계약 목록이 비어 있다는 것은 자유롭게 이름을 만들 수 있다는 뜻이 아니라 아직 승인된 공개 인터페이스가 없다는 뜻이다.
- 필요한 공개 인터페이스가 계약에 없으면 임시 이름으로 우회하지 않는다. 먼저 중앙 계약 변경안과 producer/consumer 영향을 제안한다.
- 승인된 이름을 코드·launch·config에서 사용하는 것은 가능하지만, 다른 파일을 새로운 원본 계약으로 만들면 안 된다.
- `publish_enabled: false`인 TF는 launch나 코드에서 발행하지 않는다. MORAI 원본 frame 문자열을 중앙 frame 이름처럼 사용하지 않는다.
- 센서 파생 데이터는 원본 측정 `header.stamp`를 유지한다. 처리 완료 시각으로 덮어쓰지 않으며 clock domain이 다른 시각끼리 직접 비교하지 않는다.
- MORAI UDP adapter의 node/topic/frame/port와 활성화 여부는 `config/morai_interface/udp_ros_bridge.yaml`을 따른다. `runtime_activation_allowed: false` 채널은 system bringup에 넣지 않는다.
- 계약 변경은 중앙 계약, 공유 메시지, producer, consumer, launch, config, 문서와 통합 검증을 하나의 변경 단위로 갱신한다.

## 패키지 경계

- MORAI 외부 UDP 송수신은 `morai_interface_pkg`만 수행한다.
- 공유 데이터 타입 구현은 `common_msgs_pkg`만 소유하고 그 의미는 `ros_architecture_pkg`가 승인한다.
- Camera/LiDAR 패키지는 관측값과 신뢰도·측정시각을 제공하며 각자 전역 world model을 만들지 않는다.
- 좌표 변환·시간 동기화·교차 센서 융합·동적 객체 추적은 `world_model_pkg`가 소유한다.
- `hd_map_pkg`의 정적 지도와 `global_route_manager_pkg`의 주행 진행 상태를 섞지 않는다.
- `path_planning_pkg`는 actuator 또는 UDP 명령을 직접 만들거나 보내지 않는다.
- `vehicle_control_pkg`는 nominal 제어를 만들고 `safety_supervisor_pkg`가 최종 명령을 검사·제한·거부한다.
- `runtime_evaluation_pkg`는 관측과 평가만 하며 제어 명령을 변경하지 않는다.
- 전체 실행 조합과 시작 순서는 `system_bringup_pkg`만 소유한다.

## 파일 배치

- 알고리즘 구현: 해당 패키지의 `src/`
- 런타임 파라미터: 해당 패키지의 `config/`
- 상세 설계·검증 근거: 해당 패키지의 `docs/`
- 단독 실행: 해당 패키지의 `launch/`
- 전체 시스템 실행: `system_bringup_pkg/launch/`
- 공개 계약과 아키텍처 그림: `ros_architecture_pkg/`

하드코딩 가능한 로컬 상수와 공개 인터페이스를 구분한다. 변경 가능한 임계값은 YAML에 두되, topic·node·frame 이름을 패키지 로컬 YAML에서 새로 정의하지 않는다.

## 규정 및 안전

- 허용된 대회 UDP 정보 외 Ground Truth, Bounding Box, V2I/V2V 또는 숨은 시뮬레이터 상태를 사용하지 않는다.
- GPS blackout, sensor stale, NaN, 지연, UDP 단절과 모델 실패를 정상적인 오류 경로로 처리한다.
- GPS blackout 구간에서 차로 패널티가 없더라도 차로 안전 기능을 끄지 않는다.
- 전역경로, sample scene, 체크포인트를 실제 위치 Ground Truth로 사용하지 않는다.
- `참고파일들/` 원본은 명시적인 승인 없이 수정·이름변경·정규화하지 않는다.

## 개발 및 검증

- ROS1 Noetic과 catkin을 기준으로 한다.
- `main`에서 직접 개발하지 않고 `feature/*` 브랜치를 사용한다.
- 구현 전 현재 파일, 패키지 manifest, CMake, launch와 중앙 계약을 검사한다.
- 패키지 단위 테스트와 producer-consumer 계약 테스트를 함께 작성한다.
- catkin build, launch XML, YAML, 중앙 계약 준수와 의존성 방향을 검사한다.
- TF 변경은 단일 parent, cycle 부재, 원본 extrinsic 일치, 검증되지 않은 TF의 발행 금지를 검사한다.
- Timestamp 변경은 live/replay clock 분리, 측정시각 보존, reset/역행과 watchdog 동작을 검사한다.
- launch 성공은 시작 가능성만 증명한다. 실제 UDP, topic callback, timestamp, TF, output과 MORAI closed-loop 검증을 별도로 보고한다.
