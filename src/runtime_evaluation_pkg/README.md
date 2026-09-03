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

## 디렉터리

- `config/`: 규정 버전별 metric과 report 설정
- `docs/`: metric 정의, 판정 차이와 검증 보고서
- `launch/`: Runtime Evaluation 단독 실행
- `src/`: observer, metric aggregator와 report 구현
