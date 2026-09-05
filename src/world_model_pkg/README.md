# world_model_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 이 패키지가 필요한 이유

Camera/LiDAR 결과를 각 패키지가 임의로 HD Map 위에 투영하면 서로 다른 timestamp, calibration과 pose를 사용해 Planner에서 충돌한다. 이 패키지가 지도·ego·동적 객체 융합의 단일 소유자가 된다.

## 담당 범위

- 관측 timestamp에 해당하는 ego pose history 조회와 보간
- 승인된 sensor extrinsic을 사용한 좌표 변환
- Camera/LiDAR cross-sensor association, fusion과 dynamic tracking
- 정적 HD Map, ego footprint, lane/signal/free-space와 객체의 일관된 scene 구성
- 각 객체와 layer의 source, age, confidence와 uncertainty 유지
- planner-ready local world model과 freshness/quality 상태 제공

## 담당하지 않는 범위

- 개별 센서 raw inference, ego Localization 자체
- 행동 결정, route progress, trajectory와 actuator 제어
- sample scene의 객체·신호 상태를 live world state로 주입

## 안전 원칙

- 최신 메시지끼리 단순 결합하지 않고 측정시각을 기준으로 정렬한다.
- Localization uncertainty가 증가하면 투영된 객체 uncertainty도 함께 증가시킨다.
- stale observation, frame 불일치와 calibration 누락은 명시적으로 reject/degraded 처리한다.
- 같은 객체의 Camera/LiDAR 관측을 이중 장애물로 세지 않도록 association 근거를 유지한다.

## 논리 입출력

- 입력: versioned HD Map slice, pose history/quality, timestamped Camera/LiDAR observations
- 출력: time-aligned tracked scene, occupancy/free-space, traffic context와 world-model quality

좌표 변환에는 중앙 [`TF 계약`](../ros_architecture_pkg/config/tf/frame_contract.yaml)에서 승인된 frame과 extrinsic만 사용한다. 시간 정렬에는 중앙 [`Timestamp 계약`](../ros_architecture_pkg/config/timestamp/timestamp_contract.yaml)을 적용하고, 각 관측의 source stamp를 fusion publication time으로 교체하지 않는다.

## 디렉터리

- `config/`: sync, association, tracking, uncertainty와 stale 파라미터
- `docs/`: calibration, frame, fusion schema와 평가 근거
- `launch/`: World Model 단독 실행
- `src/`: temporal buffer, transform, fusion과 tracking 구현
