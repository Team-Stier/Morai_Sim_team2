# vehicle_control_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- 승인된 trajectory의 lateral/longitudinal tracking
- nominal steering, accel과 brake 계산
- 물리 saturation, rate limit, anti-windup과 command watchdog
- tracking error, controller state와 command freshness 제공
- 정지 trajectory와 controlled stop 추종

## 담당하지 않는 범위

- 경로 탐색, 객체 융합, route progress와 신호 판단
- 최종 Safety 승인과 MORAI UDP 직렬화·송신
- 여러 독립 제어 명령 경로 생성

## 대회 규정상 유의사항

- 차량의 공지 최대 휠 조향각은 40°이고 최소 회전 반경은 5.87 m다.
- 40°가 UDP steering 필드와 같은 단위·부호·정의라고 추측하지 않는다. 실제 차량 응답으로 변환 계약을 검증한다.
- longitudinal 제어는 최종적으로 `cmd type = 1` accel/brake 계약에 맞아야 한다.
- 과도한 제어 진동과 overspeed가 패널티·경로 이탈·충돌로 이어지지 않도록 제한한다.

## 논리 입출력

- 입력: valid trajectory, ego motion state와 Safety가 제공한 운용 constraint
- 출력: timestamp와 유효기간이 명시된 nominal actuator command와 controller health

Safety Supervisor가 Controller 뒤에서 최종 gate를 수행하므로 이 출력은 아직 MORAI 송신 승인을 의미하지 않는다.

## 임시 Team1 제어기 시험

`team1_controller_test.launch`는 고정 submodule의 Team1 Pure
Pursuit/Stanley/IMM + PID 제어기를 격리해 실행한다. 이름과 달리 upstream
저장소에는 MPC 구현이 없다.

Adapter는 권위 있는 `/planning/trajectory`의 상태, 만료시각, map frame,
point와 boundary clearance를 검사한 뒤 fresh odometry stamp에 동기화된 제어기
전용 Path를 만든다. 기존 `/planning/local_path`는 계속 시각화 전용이다.

```bash
roslaunch vehicle_control_pkg team1_controller_test.launch
```

결과는 `/control_test/team1` namespace에만 남고 MORAI UDP로 전달되지 않는다.
출처, 제한, 실행 전제와 제거 방법은
[`docs/team1_temporary_integration.md`](docs/team1_temporary_integration.md)에 있다.

## 디렉터리

- `config/`: gain, saturation, rate, watchdog과 차량 모델 파라미터
- `docs/`: 제어기 설계, 식별, 단위·부호와 응답 검증
- `launch/`: Vehicle Control 단독 실행
- `src/`: tracking controller와 nominal command 구현
