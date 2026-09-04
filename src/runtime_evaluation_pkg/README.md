# runtime_evaluation_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- 출발 1분/5%, 체크포인트 순서·달성도와 15분 제한 기록
- 속도·신호·차로·충돌 이벤트와 예상 패널티 계산
- 완주율, 실제/패널티 포함 시간, 경로 이탈과 mission 결과 기록
- sensor/model latency, data age, packet drop, 제어 진동과 추론 지연 측정
- run ID, scenario, 날씨, 시간, seed, 코드·데이터 버전과 결과 연결

## 담당하지 않는 범위

- 차량 명령 수정, Safety 판단 또는 route/planning 의사결정
- 운영측 판정 프로그램을 대체하거나 자체 결과를 공식 점수로 주장
- 허용되지 않은 시뮬레이터 정보를 평가 편의를 위해 수신

## 대회 규정상 유의사항

- 완주 팀은 두 주행 중 더 짧은 총 시간으로 순위를 정한다.
- 미완주 결과는 체크포인트 달성도가 먼저이고 총 시간이 다음이다.
- 패널티 로직은 규정 버전과 함께 저장하며 운영측 변경 가능성을 표시한다.
- GPS blackout 구간의 차로 패널티 미적용과 차로 안전 필요성을 구분한다.

## 논리 입출력

- 입력: 중앙 계약에서 관측용으로 승인한 상태·event·timing 정보
- 출력: 주행에 영향을 주지 않는 metric, report와 재현 metadata

이 패키지는 command path와 독립된 read-only observer여야 한다.

## 현재 구현: planning RViz observer

`planning_visualizer_node`는 중앙 계약의 `/localization/odometry`를 읽어
RViz 전용 ego trace와 IONIQ 5 body-envelope marker를 발행한다. 공개 node,
topic, frame과 timeout의 원본 정의는 이 문서가 아니라
[`interface_contract.yaml`](../ros_architecture_pkg/config/interface_contract.yaml)이다.

- `map` parent frame과 `base_link` child frame만 수용한다.
- 위치, 자세, 속도와 covariance에 NaN/Inf가 있으면 표본을 거부한다.
- 단위 quaternion, 양의 timestamp, 엄격히 증가하는 표본만 trace에 추가한다.
- odometry가 계약 age를 넘거나 미래 시각이면 현재 vehicle marker를 삭제한다.
  marker lifetime은 매 발행에서 새로 0.25 s를 부여하지 않고 accepted
  odometry timestamp의 절대 만료 시각까지 남은 시간으로 설정한다.
  따라서 node가 예기치 않게 종료되어도 stale 차량이 남지 않는다.
- trace는 최근 2,000개 accepted pose로 제한하여 RViz 메모리와 전송량을 제한한다.
- 차량 cube는 4.635 m x 1.892 m x 2.434 m이고 rear-axle origin에서 body
  center까지 차량 x축으로 1.5275 m 이동한다.

이 출력은 시각화 전용이며 planner, controller 또는 safety 결정을 변경하지 않는다.
상세 reject 조건과 검증 범위는
[`docs/planning_visualization.md`](docs/planning_visualization.md)에 기록한다.

단독 실행:

```bash
roslaunch runtime_evaluation_pkg runtime_evaluation_pkg.launch
```

## 디렉터리

- `config/`: 규정 버전별 metric과 report 설정
- `docs/`: metric 정의, 판정 차이와 검증 보고서
- `launch/`: Runtime Evaluation 단독 실행
- `src/`: observer, metric aggregator와 report 구현
