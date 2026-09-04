# MORAI `morai/test1` 작업 상태

기준일: 2026-09-04

## 작업 범위

- 기준 브랜치: `feature/path&control_test`
- 작업 브랜치: `morai/test1`
- 대상 저장소: `Team-Stier/Morai_Sim_team2`
- 기존 경로 생성 및 제어 구현은 수정하지 않았다.
- 외부 참고 저장소에서는 센서 UDP bridge와 localization 관련 구현만 가져왔다.

## 완료한 작업

- `Team-Stier/Morai_Sim-Stier-Team2` revision
  `f889f7f5ae47b51c4c5c0211c6a92a62398ca269`의 GPS, IMU, camera,
  legacy vehicle-status, control-command UDP 구성과 dual-EKF localization 구현을
  현재 저장소의 패키지 경계와 중앙 ROS 인터페이스 계약에 맞춰 통합했다.
- 센서 bridge 및 localization용 launch/config/test와
  `system_bringup_pkg/launch/morai_sensor_localization.launch`를 추가했다.
- 기존 `path_control_test.launch` 및 경로 생성/제어 소스는 그대로 유지했다.
- Team1 controller submodule은 저장소가 지정한 revision
  `11c076b2e464697d86c76f968999cec58d0ffd69`에 맞췄다.
- MORAI에 `2026_molit_path_start_empty.json` 시나리오를 불러와 Ego를 경로
  시작 위치로 배치했다. 예기치 않은 차량 구동을 막기 위해 Simulator는
  일시정지 상태로 종료했다.
- 시나리오 로드 전 실제 UDP 수신으로 GPS, IMU, 전방/좌/우 camera bridge의
  데이터 유입을 확인했다.
- `catkin_make` 빌드와 전체 `catkin_make run_tests`를 실행했으며,
  `catkin_test_results` 기준 309 tests, 0 errors, 0 failures를 확인했다.

## 현재 제한과 미완료 항목

- 시나리오 로드 과정에서 Simulator의 network connection이 해제되어 현재
  UDP 센서 연결은 끊긴 상태다. MORAI GUI의 Network Settings와 각 Sensor
  Settings에서 기존 값을 유지한 채 `Connect`를 다시 눌러야 한다.
- 가져온 vehicle-status decoder는 legacy 229-byte/7803 규격이고 현재
  Simulator의 Competition Vehicle Status는 152-byte/9094 규격이다. 잘못된
  상태 값을 제어에 쓰지 않도록 vehicle-status와 localization은 기본 launch에서
  비활성화했다.
- 현재 `path_control_test.launch`는 의도적으로 Safety Supervisor의 최종 출력과
  MORAI UDP sender를 포함하지 않는다. 또한 실제 센서 기반
  `/world_model/lead_vehicle` producer가 없어 planner가 fail-closed 상태가 된다.
- 현재 planner 설정은 공식 경로 약 1246.4 m 지점의 혼합 차선 경계와 경로 형상
  충돌 때문에 A411→A409 전이를 비활성화하며, 이 지점에서는 의도적으로
  `no-path/stop-required`가 된다. 따라서 현 경로/제어 코드를 그대로 유지하면
  전체 2184.6 m 한 바퀴를 완주할 수 없다.
- 따라서 안전한 closed-loop 한 바퀴 주행, 주행 영상 및 한 바퀴 rosbag 저장은
  이번 작업에서 완료하지 않았다.

## 다음 실행에 필요한 작업

1. 현재 152-byte Competition Vehicle Status 규격을 검증해 canonical
   `/vehicle/twist` adapter를 마련한다.
2. 실제 센서 관측을 사용하는 world-model clear/lead 상태 producer와 Safety
   Supervisor 및 승인된 최종 명령 sender를 연결한다.
3. MORAI GUI에서 Ego network와 GPS/IMU/camera sensor network를 재연결한다.
4. 전체 topic freshness와 fail-closed 동작을 확인한 후 Simulator를 resume하고,
   한 바퀴 종료 조건과 함께 영상 및 rosbag을 기록한다.

가짜 no-lead heartbeat, Simulator ground truth 또는 Safety를 우회하는 직접 UDP
송신은 사용하지 않는다.
