# safety_supervisor_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- Controller 뒤에서 nominal command를 검사하는 최종 fail-closed gate
- stale/누락/NaN, timestamp 역행, Localization/World Model/route quality와 통신 상태 감시
- 제한속도, command range/rate, collision risk와 경로 이탈 위험 검사
- 명령 clamp, reject, controlled stop와 latched fault 정책
- 최종 명령 권한이 단 하나임을 보장하고 결정 이유를 기록

## 담당하지 않는 범위

- 정상 주행 trajectory 생성, sensor perception과 Localization
- UDP packet 직렬화와 공식 채점 판정
- 오류를 숨기기 위한 stale data 재사용

## 대회 규정상 유의사항

- 기본 60 km/h 제한을 최종 경로에서 보호한다. 예외 구간은 검증된 route context가 있을 때만 적용한다.
- GPS blackout 구간에서 차로 패널티가 없더라도 차로 이탈 안전을 비활성화하지 않는다.
- CollisionData는 이미 발생한 충돌 정보이므로 선제 회피 센서로 간주하지 않는다.
- 코드 실행 후 Operator 조작에 의존하지 않고 오류를 처리해야 한다.

## 논리 입출력

- 입력: nominal command, ego/route/world/controller/interface health와 안전 constraint
- 출력: 승인·제한·정지 중 하나의 최종 command와 machine-readable safety state/reason

Safety가 준비되지 않았거나 필수 상태가 유효하지 않으면 주행 명령을 fail-open으로 통과시키지 않는다.

## 디렉터리

- `config/`: gate, timeout, limit, stop과 fault-latch 파라미터
- `docs/`: hazard analysis, 상태 머신과 fault-injection 결과
- `launch/`: Safety Supervisor 단독 실행
- `src/`: health aggregation, command gate와 state machine 구현
