# common_msgs_pkg

> **PUBLIC INTERFACE LOCK v1.0.0:** 이 패키지는
> [`interface_contract.yaml`](../ros_architecture_pkg/config/interface_contract.yaml)에
> 이름이 예약된 공유 타입만 구현한다. 타입 이름과 field 계약을 독립 변경하지 않는다.

## 담당 범위

- 중앙 승인을 받은 공유 ROS message, service, action 타입 구현
- 필드 의미, 단위, frame, timestamp와 invalid 표현을 계약과 일치하도록 유지
- producer와 consumer가 함께 실행하는 serialization·contract test 제공

## 담당하지 않는 범위

- 인터페이스 의미와 이름의 독자적 결정
- 기능 알고리즘, UDP 송수신, launch 조합
- 패키지 한 곳에서만 쓰는 내부 자료구조의 무조건적인 공용화

## 공개 ROS 입출력

이 패키지는 **런타임 node와 topic I/O가 없다**. 빌드 시 공유 타입을 생성하는
schema provider다.

![Common Messages 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** 없음

중앙 계약에는 `ComponentStatus`, `CameraObservationArray`, `EgoState`,
`WorldModel`, `Trajectory`, `ActuatorCommand` 등 custom type 이름이 예약됐다.
현재 실제 `.msg` schema와 message generation 설정은 아직 구현되지 않았으므로
사용 가능한 타입이라고 주장하면 안 된다.

## 통합 전 자체 확인

- 새 `.msg/.srv/.action` 이름을 먼저 만들지 않고 중앙 계약을 먼저 변경한다.
- field의 timestamp, frame, unit, invalid/quality 의미를 producer·consumer와 함께 검토한다.
- 타입 변경 시 관련 모든 패키지와 contract test를 같은 PR에서 갱신한다.
- 이 패키지에 기능성 런타임 node나 topic을 추가하지 않는다.

## 디렉터리

- `config/`: 타입 검증용 로컬 설정
- `docs/`: schema 결정과 migration 근거
- `launch/`: 타입/contract 검사 실행용 placeholder
- `src/`: 생성 타입 보조 검증 코드
- `msg/`, `srv/`, `action/`: 중앙 승인 후에만 사용
