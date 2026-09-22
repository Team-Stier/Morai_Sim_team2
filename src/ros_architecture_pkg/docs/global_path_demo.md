# 전역경로만 따라가는 MORAI 실행

2026-09-21 사용자의 “지금 당장 전역경로만 따라가게 실행” 요청으로 제공한
개발용 profile이다. 기존 전체 계획·Safety 구현을 완료했다고 의미하지 않는다.
사용자가 앞서 요청한 대로 원본 제어 로직 외 별도 방어 계층은 추가하지 않았다.

실제 실행:

```bash
cd /home/paik/morai-artifacts/closedteam2-control-worktree
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch system_bringup_pkg global_path_demo.launch
```

센서와 Localization은 이미 실행된 것을 사용한다. 기본 경로는 저장소
`참고파일들/2026_molit_comp_global_path (3).txt`다. source workspace 밖에서
실행할 때는 `path_file:=/absolute/path/to/route.txt`를 지정한다.
고정 속도 규칙은 `config/map/course_speed_policy.yaml`에서 읽는다.
감속·곡선 속도와 앞뒤 경로 길이는 `path_planning_pkg/config/global_path_demo.yaml`에서 설정한다.

흐름: global_route_manager_node의 map 전역경로 → path_planner_node의 odom
local trajectory → vehicle_controller_node → 개발용 safety_supervisor_node 단순 전달
→ morai_control_sender → 127.0.0.1:9093.

Localization EgoState와 Odometry의 동일 stamp 쌍으로 map 경로를 odom에 옮긴다.
경로 좌표를 ego 위치 측정값으로 사용하지 않는다. 현재 개발 Localization의
stop_required는 이 profile에서 Controller에 의해 무시하지만 local_odometry_valid는
유지한다. 정상 Controller launch의 해당 옵션 기본값은 false다. Safety 노드도
이 profile에서는 단순 전달이며 일반 Safety 검사를 구현한 것이 아니다.

실제 MORAI Network Settings에서 Cmd Control Host PORT 9093을 확인하고 연결했다.
55-byte cmd_type=1, ctrl_mode=2, Drive 명령이 수신되며, brake=1에서 normalized
steer=0.25를 보내 차량 UI에 10.00°가 표시됐다. 후속 양의 조향에서 허용된
GPS/IMU 기반 yaw가 증가해 좌회전 부호를 확인했다. 전달 배율은 rad/(40°)다.
별도 default UDP 계약은 변경하지 않고 중앙 global_path_demo 모듈에 시험 예외를 둔다.

관측: 차량이 시작점 근처 경로에서 약 5 m 벗어난 상태로 합류한 뒤, 첫 관측
구간의 Stanley 전륜 기준 오차는 약 0.03~0.10 m, 속도는 약 10 km/h였다.
MORAI UI에서도 9.94 km/h와 차량 이동을 확인했다. 이 최초 기록은 저속 합류
시험이다. 이후 속도 적용 관측은 [코스 속도 검증 기록](course_speed_policy.md)을
따른다. 교통 판단은 포함하지 않는다.

기록은 `/home/paik/morai-artifacts/global-path-demo-*.jsonl`,
`global-path-demo-live.log`, `global-path-demo-running.png`에 저장했다.

중지하려면 MORAI Vehicle Controller를 Manual-Keyboard로 바꾸고 S로 제동한다.
외부 명령이 Manual 선택을 다시 Auto로 바꾸지 않도록 demo roslaunch도 종료한다.
현재 프로세스를 정지하고 전자 제동하려면 publisher를 먼저 종료한 후 기존
morai_interface serializer의 accel=0/brake=1/steer=0 패킷을 수동 송신할 수 있다.
