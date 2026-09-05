# 전체 시스템 아키텍처

## Nominal data/control 흐름

![Nominal data/control 아키텍처](system_nominal_flow.svg)

- Mermaid 원본: [`system_nominal_flow.mmd`](system_nominal_flow.mmd)
- PNG 이미지: [`system_nominal_flow.png`](system_nominal_flow.png)

센서·지도 입력부터 인식, Localization, Route, World Model, Planning, nominal
control, Safety 최종 gate와 MORAI 제어 송신까지의 경로다.

## Health/readiness/safety/evaluation 흐름

![Health/readiness/safety/evaluation 아키텍처](system_health_safety_flow.svg)

- Mermaid 원본: [`system_health_safety_flow.mmd`](system_health_safety_flow.mmd)
- PNG 이미지: [`system_health_safety_flow.png`](system_health_safety_flow.png)

component status와 collision event, upstream readiness, 최종 Safety 판단과
read-only Runtime Evaluator의 metric 경로를 표시한다. Evaluator가 성능 측정을
위해 구독하는 모든 data-topic 간선은 전체 상세본에서 확인한다.

두 읽기용 그림은 중앙 `interface_contract.yaml`의 `architecture_views`에서 자동
생성한다. 각 상자는 관련 exact node, topic과 message type을 표시하고 필요한
MORAI sensor/event UDP 입력 및 Ego Ctrl Cmd UDP 출력 경계도 포함한다.
초록·파랑은 현재 live transport가 확인된 경계이고 회색·주황은
예약·미구현·비활성 경계다.

각 패키지 I/O 그림의 패키지 역할과 node·topic 한국어 설명도 중앙 계약의
`diagram_summary_ko`, `diagram_description_ko`에서 함께 생성된다. Mermaid,
SVG와 PNG는 읽기용 투영이므로 직접 수정하지 않는다.

## 전체 exact graph

- [24개 node·35개 topic 전체 상세 SVG 확대해서 열기](system_architecture.svg)
- [전체 상세 Mermaid 원본](system_architecture.mmd)
- [전체 상세 PNG](system_architecture.png)

전체 graph는 모든 producer-consumer 연결을 한 장에 보존한 감사용 상세본이다.
패키지 단위 구현에서는 각 `src/<package>/docs/interface_io.svg`를 함께 사용한다.

## 모듈형 Planning 경계

`path_planning_pkg` / `path_planner_node`는 다음 세 그룹의 승인된 공개
데이터만 입력으로 사용한다.

- Localization: local odometry, map-referenced ego state와 위치추정 상태
- Route: 다음 경로 구간, 진행도, 체크포인트·속도 정보와 route 상태
- World Model: 측정시각과 좌표계가 정렬된 통합 scene과 융합 상태

Planner는 behavior planning과 local motion planning을 수행해 유효기간과 시간이
명시된 `/molit/planning/trajectory`를 유일한 주행 데이터로 출력한다. 입력
freshness, 재계획 상태, 계산 지연, 준비 여부와 trajectory 유효성은
`/molit/planning/status`로 보고한다. Controller가 trajectory를 추종해 nominal
command를 만들고 Safety Supervisor가 최종 gate를 수행한 후에만 MORAI sender로
전달한다.

Camera/LiDAR 원본과 개별 Perception 관측은 Planner가 직접 구독하지 않는다.
Camera/LiDAR Perception은 센서 관측을, World Model은 시간·좌표 정렬과
cross-sensor fusion 및 tracking을 각각 소유한다. 위 경계와 이름은 설계 승인
상태이며, Planner와 trajectory schema는 아직 구현·runtime 검증되지 않았다.

## 설계 결정

1. Camera와 LiDAR 인식 결과를 Planner가 직접 조립하지 않는다.
2. `world_model_pkg`가 관측 시각의 ego pose history와 승인된 calibration을 사용해 좌표 변환, 시간 정렬, 교차 센서 융합과 추적을 수행한다.
3. Planner는 Localization, Route와 하나의 일관된 World Model만 공개 입력으로
   사용하고 raw sensor나 개별 Perception 관측을 직접 조립하지 않는다.
4. Controller는 nominal 명령만 만들고 Safety Supervisor가 Controller 뒤에서 최종 명령을 gate한다.
5. 외부 MORAI 통신은 단 하나의 interface package를 통과한다.
6. 평가 패키지는 read-only이며 제어 경로에 연결하지 않는다.
7. 공개 경계만 고정한다. 내부 node와 topic은 private name 또는
   `/molit/internal/<package>/...` 아래에서 자유롭게 설계한다.
8. 전역 경로와 World Model은 `map`, 제어용 local trajectory는 연속적인
   `odom` frame을 사용한다. Local Odometry는 절대 위치 Ground Truth가 아니다.
9. `/molit/system/readiness`는 Safety 이전 upstream 준비 상태이고, 최종 주행
   허용 여부는 `/molit/safety/state`가 나타낸다.
10. `path_planner_node`는 v1에서 `/molit/planning/trajectory`만 출력하며
    actuator 또는 MORAI UDP를 직접 출력하지 않는다.

## HD Map 완료 정의

현재 제공된 4,430점 전역경로는 HD Map이 아니다. HD Map을 완료로 선언하려면 최소한 다음 레이어와 검증 근거가 필요하다.

- 좌표계, 원점, 지도 버전과 원본 해시
- 차선 중심선·경계선·실선·중앙선·차선 종류
- 도로·차선 연결 관계와 주행 가능 영역
- 정지선·횡단보도·교차로·신호등 연결
- 속도 제한과 공식 예외 구역
- Localization용 정적 landmark 또는 map-matching 표현
- 공식 전역경로·체크포인트·Link ID 대응

공식 맵명과 sample scene 맵명이 다른 문제, 원본 지도 사용 허가와 실제 시뮬레이터 로드 결과가 해결되기 전에는 “완벽한 HD Map”이라고 표현하지 않는다.

## 시간 정렬 원칙

센서별 최신 메시지를 단순히 한 시점의 값처럼 합치지 않는다. World Model은 각 관측의 측정 timestamp에 해당하는 ego pose를 사용하고, 보간 가능 범위·최대 age·불확실성 전파 기준을 중앙 계약으로 정의해야 한다.

## Calibration 소유권

- `ros_architecture_pkg`: frame tree, transform 방향, 단위와 활성 calibration version 승인
- `camera_perception_pkg`: camera intrinsic/extrinsic 후보 파일과 검증 근거 소유
- `lidar_perception_pkg`: LiDAR extrinsic/time offset 후보 파일과 검증 근거 소유
- `localization_pkg`: GPS/IMU 장착 관계와 estimator에서 사용하는 시간 보정 근거 소유
- `world_model_pkg`: 승인된 calibration을 소비하며 값을 자체 수정하거나 별도 원본으로 복사하지 않음

여러 패키지에 영향을 주는 calibration 변경은 중앙 계약 version, 관련 sensor 설정, World Model 회귀 테스트와 시각 정합 결과를 같은 변경 단위로 갱신한다.
