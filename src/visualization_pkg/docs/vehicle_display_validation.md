# 차량 표시 구현·검증 (2026-09-10)

로컬 `PAIK`에서 차량 표시 노드, RViz 설정, 단독 launch와 중앙 consumer 경계를
구현했다. 원격 commit·push·PR 작업은 수행하지 않았다.

## 확인 결과

- 전체 catkin 빌드 성공.
- 시각화 pure odom/geometry 5개, ROS marker 동작 7개, producer-consumer/launch 2개 통과.
  Catkin의 rostest 실행 항목을 포함한 집계는 15 tests, 오류·실패·skip 0이다.
- 중앙 아키텍처 44개 중 43개 통과, 과거 `/home/stier` 센서 프로필 부재 1개 skip.
  현재 GPS/IMU 프로필 비교는 통과했다.
- Localization 회귀 검사 통과(Catkin 집계 15 tests, 오류·실패·skip 0).
- YAML/RViz/XML 파싱과 중앙 다이어그램 `--check`, diff 공백 검사 통과.

합성 pose `(10,20,3)`에 yaw 90도와 body offset `(1,0,0.5)`를 적용한 중심이
`(10,21,3.5)`가 되는지 검사했다. 원본 nanosecond stamp, map/odom 구분,
invalid·stale·reset·멈춘 clock에서 도형 삭제, 부분 pose 투영과 표시를 검사했다.

별도 ROS master에 합성 EgoState/LocalizationStatus를 넣고 실제 RViz에서
사각형·전방 화살표·정지/offset 안내 표시를 확인했다. 미리보기 창에는
`SYNTHETIC INPUT PREVIEW / NOT LIVE LOCALIZATION`을 명시했다.
테스트 master와 입력 publisher는 종료했다. 실제 센서 세션에는 테스트 pose를 넣지 않았다.

## 실제 시뮬레이터 연결 범위

실제 ROS master에서는 `vehicle_visualizer_node`가 Localization의 세 승인 topic을
구독하고, 내부 MarkerArray를 `vehicle_rviz`가 받는 연결을 확인했다.
현재 Localization의 유효한 EgoState/Odometry 발행이 없으므로 실시간 차량
사각형은 표시하지 않고 `WAITING FOR LOCALIZATION`으로 대기한다.

TF를 발행하지 않았으며 `map -> odom -> base_link`나 센서 TF 잠금을 해제하지 않았다.
현재 RViz의 Fixed Frame 관련 Global Status 경고는 TF 부재 때문이다.
같은 reference frame에 직접 만든 마커의 렌더링은 확인했다.

본 검증은 표시 코드·geometry 합성의 검증이다. 실제 차량 위치 정확도,
`base_link`와 활성 MORAI pivot의 동일성, 박스의 수직 중심 및 주행 준비 완료를
증명하지 않는다. 차체 footprint 후보·MORAI 치수의 근거는
[중앙 시각화 문서](../../ros_architecture_pkg/docs/visualization_vehicle.md)를 따른다.
