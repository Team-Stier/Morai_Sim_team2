# RViz 현재 플래너 표시 계약

사용자 요청: RViz Interact 도구 아래에서 현재 선택된 플래너를 구분한다.
기존 `/molit/planning/status`의 읽기 전용 consumer로 `vehicle_visualizer_node`를
추가한다. 공개 구독은 `vehicle_visualizer_node`가 담당하고 원본 상태를
`/molit/internal/visualization/planner_status`로 전달해 RViz가 읽는다.
visualization 패키지 입력 목록에도 공개 상태 구독을 등록한다. Producer는 기존
`path_planner_node`, 타입은 `common_msgs_pkg/ComponentStatus` 그대로다.
새 공개 topic, node, 공유 메시지 또는 제어 경로는 추가하지 않는다.

RViz Display plugin은 `reason`의 기존 `planner_mode`와 `zone` 텍스트를
표시 용도로만 읽는다. `hybrid_astar`는 `Hybrid A*`, `frenet`은
`Frenet (RDDF)`다. Lanelet은 현재 planner mode 이름이 아니므로 별도 모드로
표시하지 않는다. `stop_required`는 `STOP` 표시에만 쓰며 제어 판단은 하지 않는다.
reason은 사람이 읽는 진단 텍스트이며 이 표시 형식을 제어용 계약으로 승격하지 않는다.
모드 텍스트가 없으면 `WAITING`, 모르는 모드는 `UNKNOWN`이다.

frame/좌표 변환은 없다. 기존 상태 생성 `header.stamp`를 읽고 발행하거나
덮어쓰지 않는다. 전달은 원본 header/stamp를 보존하고 RViz 구독 queue는 1이며, 새 상태가 없으면 화면상의
`Display timeout`(기본 2초) 뒤 `STALE`을 붙인다. 수신 시 오래된 latched
상태도 생성시각으로 판정한다. 수신 이후 경과 시간은 monotonic clock으로
확인하므로 표시 watchdog이 ROS clock 정지로 멈추지 않는다. 이 timeout은
주행 freshness/watchdog 기준을 변경하지 않는 UI 파라미터다.

영향: visualization의 RViz plugin, 의존성, 기본 RViz config, README와
producer-consumer 계약 테스트 및 중앙 입력 다이어그램을 함께 갱신한다.
기존 planner producer, controller, safety consumer의 동작은 동일하다.

## 검증 (2026-10-05)

- Noetic catkin build/install 성공, RViz plugin 검색 확인.
- 플래너 표시 문자열 gtest 2개, 시각화 단위/계약 테스트 18개,
  상태 원본 stamp 전달을 포함한 차량 표시 rostest 8개 통과.
- 중앙 공개 인터페이스 테스트 29개, YAML/launch/plugin XML 검사,
  다이어그램 생성 및 `generate_interface_diagrams.py --check` 통과.
- 별도 ROS master와 실제 RViz에서 Hybrid A*/Frenet/WAITING/STALE 표시 확인.
  화면 근거는 저장소 밖 `/home/paik/morai-artifacts/rviz-planner-status-20261005/`에 저장.
- 전체 시각화 검사에서는 LiDAR pipeline의 관측 수신 assertion 실패가 남는다.
  이번 표시는 LiDAR 검출 로직을 변경하지 않았다. TF/timestamp 검사에서는
  로컬 MORAI 저장 프로필 SHA-256 불일치가 남는다. 모두 통과한 것으로 보고하지 않는다.
- 실제 MORAI closed-loop 주행이나 현재 실행 중인 주행 스택의 재시작은 수행하지 않았다.

차량 표시 rostest는 지도 렌더링을 검증하는 fixture가 아니므로 `show_hd_map=false`를
명시해 대형 지도 로딩이 구독 연결 제한시간을 넘는 문제를 피한다.
새 내부 상태 출력과 공개 상태 입력은 런타임 경계 assertion에 포함한다.
