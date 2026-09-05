# ros_architecture_pkg

> **CENTRAL CONTRACT AUTHORITY**
>
> 이 패키지는 전체 시스템의 공개 ROS 계약을 정하는 유일한 기준이다. 다른 패키지는 이곳에서 승인되지 않은 node, topic, message, service, action, TF frame, 단위, timestamp, 주기, queue, timeout 또는 상태 이름을 만들면 안 된다.

## 역할

- 패키지 책임과 허용 의존성 방향 정의
- 공개 ROS 인터페이스와 소유 producer/consumer 승인
- 좌표계, frame tree, 시간 기준과 단위 승인
- 상태·오류·degraded·fail-closed 의미 정의
- 전체 launch 순서와 준비 조건 승인
- 인터페이스 변경 절차 및 통합 검증 기준 관리

각 기능 알고리즘, MORAI UDP 구현, 객체 인식, 위치 추정, 경로 계획과 차량 제어는 이 패키지의 책임이 아니다.

## 현재 상태

TF frame 이름과 parent-child 구조, timestamp 의미는 첫 중앙 계약으로 승인했다. 센서 위치는 원본 근거와 ROS 변환 후보를 기록했지만 차량 원점·축을 live MORAI로 검증하기 전까지 실제 TF 발행은 잠겨 있다.

공개 ROS 경계 v1.0.0으로 **공개 경계 node 22개, MORAI 내부 node 2개와
공개 topic 34개**의 이름, owner, producer, consumer와 message type 이름을
등록했다. 이 중 실제 runtime 구현이
확인된 것은 MORAI Camera 3개와 GPS 수신 어댑터뿐이다. 나머지는
`reserved_not_implemented` 또는 disabled/prohibited 상태이며, custom message는
이름만 예약되고 `.msg` schema는 아직 구현되지 않았다. 내부 LiDAR packet topic
1개는 공개 topic 수에서 제외한다.

다른 패키지를 구현하다 공개 인터페이스가 필요하면 다음 순서로 진행한다.

1. 논리 입력·출력과 필요 이유를 이 패키지의 `docs/`에 제안한다.
2. producer, consumer, 단위, frame, timestamp, rate와 timeout을 함께 검토한다.
3. `config/interface_contract.yaml`을 먼저 갱신한다.
4. 필요한 공유 타입을 `common_msgs_pkg`에 구현한다.
5. producer·consumer·launch·config·문서·계약 테스트를 같은 변경 단위로 갱신한다.

## v1 모듈형 Planning 경계

- 공개 경계는 `path_planning_pkg` / `path_planner_node`를 유지한다.
- Planner는 Localization, Route와 time-aligned World Model의 승인된 공개
  topic만 입력으로 사용하는 behavior/motion planner다.
- Camera/LiDAR 원본과 개별 Perception 관측은 각 센서·인식 패키지가 소유하고,
  좌표 변환·시간 정렬·융합 결과는 World Model을 통해 Planner에 전달한다.
- v1의 유일한 주행 출력은 `/molit/planning/trajectory`다. Planner의
  직접 actuator/UDP 출력은 금지하며 `vehicle_controller_node →
  safety_supervisor_node → morai_control_sender`를 반드시 거친다.

이 항목은 승인된 설계 방향이지 구현 증거가 아니다.
`path_planner_node`와 `common_msgs_pkg/Trajectory` schema는 현재 모두
예약·미구현 상태다.

## 주요 문서

- [전체 시스템 아키텍처](docs/system_architecture.md)
- [Nominal data/control 읽기 이미지](docs/system_nominal_flow.svg)
- [Health/readiness/safety/evaluation 읽기 이미지](docs/system_health_safety_flow.svg)
- [전체 상세 Mermaid 원본](docs/system_architecture.mmd)
- [SVG 전체 상세 이미지](docs/system_architecture.svg)
- [공개 I/O 다이어그램 생성·검사](docs/interface_diagram_generation.md)
- [다이어그램 MMD/SVG/PNG 해시 manifest](docs/interface_diagram_manifest.json)
- [인터페이스 변경 절차](docs/interface_governance.md)
- [파트 및 패키지 소유권](docs/part_ownership.md)
- [중앙 계약 파일](config/interface_contract.yaml)
- [TF 구조와 검증 상태](docs/tf/README.md)
- [TF frame 계약](config/tf/frame_contract.yaml)
- [센서 extrinsic 계약](config/tf/sensor_extrinsics.yaml)
- [Timestamp 정책](docs/timestamp/README.md)
- [Timestamp 계약](config/timestamp/timestamp_contract.yaml)
- [MORAI UDP → ROS 계약](config/morai_interface/udp_ros_bridge.yaml)
- [패키지 레지스트리](config/package_registry.yaml)

## 공개 ROS 입출력

이 패키지는 **런타임 node와 topic I/O가 없는 governance package**다.

![ROS Architecture 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** 없음

두 읽기용 시스템 view와 전체 상세 그림에는 관련 exact node/topic/type 및
필요한 MORAI 외부 경계가 표시된다. 전체 producer-consumer 연결은 전체 상세
SVG가 보존한다. 회색/점선은 이름만 예약된 경계이며 현재 실행 가능하다는 뜻이
아니다.

각 패키지 I/O 그림의 한 줄 역할과 입력·출력 한국어 설명도 중앙 계약의
`diagram_summary_ko`, `diagram_description_ko`에서 자동 생성된다. 따라서
생성된 Mermaid나 이미지를 직접 편집하지 않는다.

## 패키지 개발자가 지킬 통합 절차

1. 루트 `AGENTS.md`, 이 README, 대상 패키지 README와
   `config/interface_contract.yaml`을 먼저 읽는다.
2. 통합 launch에서는 root namespace `/` 아래에서 `package_boundaries`의
   node basename을 정확히 사용하고 anonymous name 또는 공개 remap을 사용하지 않는다.
3. 입력·출력 topic과 type을 그대로 사용한다. 내부 구현은 private name 또는
   `/molit/internal/<package>/...` 아래에서 자유롭게 구성한다.
4. 공개 경계를 바꾸려면 중앙 계약을 먼저 수정하고 모든 producer/consumer,
   README, 다이어그램과 검사를 같은 PR에서 갱신한다.
5. 다음 명령으로 계약과 생성 문서의 동기화를 확인한다.

   `python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py --check`

## 디렉터리 원칙

- `config/`: 기계 판독 가능한 중앙 계약과 패키지 레지스트리
  - `config/tf/`: frame tree와 센서 extrinsic
  - `config/timestamp/`: clock domain과 stamp 전파 정책
  - `config/morai_interface/`: UDP channel, ROS node/topic/frame와 활성화 gate
- `docs/`: 아키텍처, ADR, 책임 경계와 통합 근거
  - `docs/tf/`: TF 근거와 활성화 게이트
  - `docs/timestamp/`: live/replay timestamp 운용 방법
- `launch/`: 이 패키지만 독립 확인할 때 사용하는 launch. 전체 시스템 bringup은 `system_bringup_pkg`가 소유
- `src/`: 향후 계약 검사 도구만 허용. 기능 알고리즘은 두지 않음
