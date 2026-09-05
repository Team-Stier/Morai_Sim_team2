# system_bringup_pkg

> **PUBLIC INTERFACE LOCK v1.0.0:** 아래 node/topic/type은
> [`interface_contract.yaml`](../ros_architecture_pkg/config/interface_contract.yaml)의
> 읽기용 투영이다. 통합 시 정확히 일치해야 하며 이 README에서 독립 변경하지 않는다.

## 담당 범위

- 전체 시스템 launch 조합의 단일 소유권
- 파라미터 파일 연결, 시작 순서와 Safety 이전 upstream readiness 집계
- 중복 publisher, 잘못된 실행 모드와 필수 패키지 누락 방지
- 실시간 MORAI 모드와 offline replay 모드 분리

## 반드시 지킬 것

- 개별 패키지 launch는 해당 기능만 시작한다.
- MORAI와 rosbag/replay 공급자를 동시에 시작하지 않는다.
- Safety Supervisor가 준비되기 전에 주행 명령을 활성화하지 않는다.
- 각 runtime profile은 required/optional 채널을 명시하고, 중앙 UDP 계약에서
  `runtime_activation_allowed: false`인 채널을 required로 지정하지 않는다.
- `ros_architecture_pkg/config/tf/`에서 `publish_enabled: true`로 승인된 정적 TF만 단일 publisher로 시작한다.
- Live MORAI와 rosbag replay의 `use_sim_time` 정책을 섞지 않는다.
- 실행 이후 Operator 조작 없이 상태 확인과 fail-closed 종료가 가능해야 한다.

GPS blackout은 정상 운용 조건이다. GPS stale 또는 no-fix만으로 즉시 정지하지 않고,
fresh Local Odometry와 승인된 uncertainty 범위 안의 Localization quality를 함께 판단한다.
마지막 GPS fix를 현재 위치 정답처럼 재사용하지 않는다. 구체 required 채널 집합과
uncertainty/timeout 수치는 측정 근거가 있는 runtime profile에서 별도로 승인한다.

현재 launch 파일과 `system_readiness_node` 구현은 아직 비어 있다. 공개 이름은
승인됐지만 모든 정적 TF 발행은 계속 잠겨 있다.

## 공개 ROS 입출력

![System Bringup 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** `system_readiness_node`

| 구분 | Topic | Type |
|---|---|---|
| 입력 | `/molit/interface/status` | `common_msgs_pkg/InterfaceStatus` |
| 입력 | `/molit/map/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/perception/camera/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/perception/lidar/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/localization/status` | `common_msgs_pkg/LocalizationStatus` |
| 입력 | `/molit/route/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/world_model/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/planning/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/control/status` | `common_msgs_pkg/ControllerStatus` |
| 출력 | `/molit/system/readiness` | `common_msgs_pkg/SystemReadiness` |

공유 custom type은 이름만 예약됐고 실제 `.msg` schema는 아직 구현되지 않았다.
`/molit/system/readiness`는 Safety를 제외한 상류 필수 구성요소의 준비 상태다.
최종 주행 허용 여부는 순환 구독 없이 `safety_supervisor_node`가
`/molit/safety/state`로 결정한다.

## 통합 전 자체 확인

- readiness 공개 노드 이름은 정확히 `system_readiness_node`를 사용한다.
- 전체 launch가 같은 공개 노드를 중복 실행하거나 공개 topic을 remap하지 않는지 확인한다.
- 상류 필수 component가 unknown/fault이면 readiness를 false로 유지한다.
- 내부 topic은 `/molit/internal/system_bringup/...`만 사용한다.
- 중앙 계약 생성 검사와 launch 중복-publisher 검사를 통과시킨다.

## 디렉터리

- `config/`: 시스템 조합과 모드별 파라미터
- `docs/`: startup sequence, readiness와 운영 절차
- `launch/`: 승인된 전체 시스템 조합
- `src/`: 향후 readiness 보조 도구
