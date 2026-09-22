# Closedteam2 제어 연결 검증

2026-09-21, Ubuntu 20.04 / ROS1 Noetic / system Python 3, catkin_make.
기준 main: `3e5c738`, 원본 Closedteam2: `ff8eb424585183790165ef2f50042ec7c8517e4b`.

- 전체 catkin build 및 install: 통과.
- Vehicle Control: 원본 조향 74개, 새 변환/PI 6개, 경계/원본 일치 5개 단위시험 통과.
- ROS 연결: 실제 Controller 노드와 테스트 producer/consumer에서 가속·감속,
  빈 정지 궤적, 상류 stop 상태 전달과 DEAD_RECKONING 입력 확인.
- Common Messages 39개, 중앙 아키텍처 46개, bringup 7개 검사 통과.
- launch/XML/YAML parse, 패키지 의존성 cycle 부재 확인.
- `generate_interface_diagrams.py --check`: 통과. Mermaid CLI 11.16.0으로
  원본 스크립트를 실행해 SVG/PNG/manifest를 갱신하고 Controller I/O 그림 확인.
- install 환경에서 Controller launch 해석과 생성 메시지 import 확인.
- 원본 조향 파일·시험의 SHA-256 및 PI 클래스 본문 일치 확인.

재현 명령:

```bash
source /opt/ros/noetic/setup.bash
PYTHONNOUSERSITE=1 catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j4 -l4
PYTHONNOUSERSITE=1 catkin_make run_tests_vehicle_control_pkg run_tests_common_msgs_pkg run_tests_ros_architecture_pkg run_tests_system_bringup_pkg -j4 -l4
catkin_test_results --all build/test_results
python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py --check
PYTHONNOUSERSITE=1 catkin_make install -DPYTHON_EXECUTABLE=/usr/bin/python3 -j4 -l4
```

실제 MORAI UDP/closed-loop 및 조향 부호·스케일 튜닝은 수행하지 않았다.
Planner/Safety/Readiness 런타임 구현은 이 변경 범위 밖이다.
사용자 지시에 따라 별도 freshness/reset/NaN 방어 계층을 추가하거나 해당
동작을 검증했다고 주장하지 않는다. 원본 제어기의 검사·제한만 유지한다.
