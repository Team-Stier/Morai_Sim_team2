# global_route_manager_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 코드와 launch에서는 계약이 승인한 이름만 사용하며, 이 문서는 그 계약의 새로운 정의원이 아니다.

## 담당 범위

- 공식 전역경로와 체크포인트 순서 관리
- ego pose를 이용한 단조롭고 연속적인 route progress 추정
- 다음 경로 구간, local route corridor와 high-level route context 제공
- 시작 1분/5%, 체크포인트 반경 3 m와 순차 통과 상태의 관측 지원
- HD Map topology와 규정 속도 구간의 대응 관리

## 담당하지 않는 범위

- 정적 HD Map 원본 소유, 동적 장애물 회피와 local trajectory
- pose 추정, actuator 또는 UDP 명령
- 실제 판정 프로그램을 대체하는 공식 채점 판정

## 현재 경로 유의사항

- 원본은 4,430점 폐곡선이며 연속 중복점 38개를 포함한다.
- 0 길이 구간의 yaw와 progress 계산을 안전하게 처리하고 원본/필터 후 인덱스 대응을 보존한다.
- 원본의 점 수·SHA-256·길이·폐곡선을 시작 시 검증하고 다르면 노드를 시작하지 않는다.
- KATRI HD Map에 투영해 검증한 40개 link 순서를 사용한다. 60 km/h 예외는 `A2256W000411` 시작에서 `A2256W000153` 종료까지만 true다.
- checkpoint 좌표는 progress 검증 기준이지 Localization Ground Truth가 아니다.

## 논리 입출력

- 입력: 중앙 계약의 localization odometry
- 출력: 4,430점 global path, route validity/progress, 현재·horizon HD Map link, 속도 예외 구간 context

시간 초과, frame 불일치, NaN/Inf, 비정상 quaternion, 6 m 초과 off-route에서는
`valid=false`로 fail-closed한다. 이 때 link horizon을 비우고 속도 예외 flag를
끄며, 직전의 정상 progress 상태를 갱신하지 않는다. 유효한 context의
header는 받아들인 odometry 측정 시각, 유효하지 않은 context의 header는
해당 `valid=false` context 발행 시각을 사용한다.

## 실행

ROS1 Noetic catkin workspace에서:

```bash
roslaunch global_route_manager_pkg global_route_manager_pkg.launch
```

기본 launch는 `hd_map_pkg/config/map_conversion.yaml`의
`references.simulator_global_path`를 통해 source workspace의 변경하지 않는
`참고파일들/` 전역경로를 찾는다. install/deploy 환경에서는 검증된
복사본을 지정한다.

```bash
roslaunch global_route_manager_pkg global_route_manager_pkg.launch \
  route_file:=/absolute/path/to/2026_molit_comp_global_path.txt
```

자세한 로더·matcher·link 구간 설계와 런타임 검증 항목은
[`docs/route_progress_design.md`](docs/route_progress_design.md)에 있다.

## 디렉터리

- `config/`: progress, projection, checkpoint와 route-local 설정
- `docs/`: 경로 provenance, topology 대응과 검증 보고서
- `launch/`: Global Route Manager 단독 실행
- `src/`: ROS 없이 검증 가능한 route loader, matcher와 progress 구현
- `scripts/`: 중앙 ROS 계약에 연결하는 얇은 ROS1 wrapper
- `test/`: 원본 무결성과 fail-closed/monotonic 동작 단위 테스트
