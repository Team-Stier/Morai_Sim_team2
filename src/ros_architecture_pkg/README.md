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

node, topic, message, service, action과 일반 parameter 목록은 여전히 비어 있다. 이는 자유롭게 이름을 정해도 된다는 뜻이 아니라, UDP 패킷 명세와 producer-consumer 검토 전까지 **아직 승인된 해당 공개 ROS 이름이 없다는 뜻**이다.

다른 패키지를 구현하다 공개 인터페이스가 필요하면 다음 순서로 진행한다.

1. 논리 입력·출력과 필요 이유를 이 패키지의 `docs/`에 제안한다.
2. producer, consumer, 단위, frame, timestamp, rate와 timeout을 함께 검토한다.
3. `config/interface_contract.yaml`을 먼저 갱신한다.
4. 필요한 공유 타입을 `common_msgs_pkg`에 구현한다.
5. producer·consumer·launch·config·문서·계약 테스트를 같은 변경 단위로 갱신한다.

## 주요 문서

- [전체 시스템 아키텍처](docs/system_architecture.md)
- [편집 가능한 Mermaid 원본](docs/system_architecture.mmd)
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
