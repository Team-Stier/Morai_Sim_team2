# LiDAR 계약 점검 및 RViz 검증 (2026-09-11)

## 적용 범위

가져온 검출 패키지와 기존 미커밋 공유 메시지/계약 변경을 함께 통합했다.
최신 origin/main에서 feature/public-interface-contract-lidar-visualization을 만들고
기존 검출 이식 커밋과 사용자 승인한 미커밋 변경을 보존해 적용했다.

- 검출 입력 PointCloud2/Bool, 출력 LidarObservationArray/ComponentStatus,
  node basename, lidar_link, 원본 scan stamp, queue와 상태 heartbeat는 중앙 계약과 일치한다.
- 검출기는 ROI/VoxelGrid/DBSCAN 객체 geometry만 구현한다. 지면, free-space,
  occupancy, 속도와 전역 tracking은 제공하지 않는다.
- calibration/freshness 미검증 flag, ready=false, stop_required=true를 유지한다.
- 시각화 consumer와 내부 MarkerArray를 중앙 계약 및 timestamp registry에 추가했다.
- TF 장착 위치는 사용자 고정값 `(2, 0, 1.5) m`, 회전은 기존 승인 영 회전이다.
  검출 좌표에 장착 위치를 중복 더하지 않고 scan-time TF를 RViz에 적용한다.
- 기존 TF 파일의 LiDAR 근거 오참조와 누락된 GPS/IMU 승인 항목을 복구했다.
  GPS/IMU 승인 기록은 git abe8b1a에서 복구하고, 현지 저장 프로필의 동일한
  위치/각도/ID/frame/period를 재확인했다. 갱신된 원본 SHA와 이전 SHA를 함께 기록했다.
  원본 센서 프로필과 참고파일은 수정하지 않았다.
- 오래된 활성화 이전 기준을 검사하던 테스트는 현재 승인된 개발 TF/IMU 범위와 맞췄다.

## 자동 검증

ROS1 Noetic, /usr/bin/python3, catkin_make -j4 빌드 통과.
common_msgs_pkg, lidar_perception_pkg, visualization_pkg, ros_architecture_pkg,
system_bringup_pkg, morai_interface_pkg 총 189개 테스트를 실행해 통과했다.
전체 저장된 catkin 결과: 262 tests, 0 errors, 0 failures (다른 패키지의 기존 결과 포함).

새 시각화 테스트는 다음을 확인한다.

- PointCloud2 → 실제 C++ detector → 공개 관측 → 시각화 마커 연결
- TF 미도착 시 보류, TF 수신 뒤 동일 scan stamp로 표시 재시도
- 프레임/중심/stamp 유지, 장착 offset 중복 적용 없음, frame_locked=false
- NaN, 잘못된 frame, 미래/중복 시각, invalid 관측 거부
- 빈 검출에 이전 박스 삭제, stale/wall watchdog/clock reset 처리

합성 ROS 시험은 별도 rostest master에서 실행했다. 실제 센서 master에는 합성 데이터를 넣지 않았다.
launch XML, YAML/중앙 계약/의존성 검사를 통과했고,
generate_interface_diagrams.py --check와 git diff --check도 통과했다.
Mermaid CLI 11.16.0은 임시 Node 22.14.0으로 실행했다 (시스템 Node는 변경하지 않음).

## 실제 MORAI 실행 확인

[10초 읽기 전용 측정](evidence/lidar_live_20260911.json): GPS 49건, IMU 288건,
LiDAR points 0건, LiDAR status=False. 실제 base_link → lidar_link TF는
translation `(2, 0, 1.5)`, quaternion `(0, 0, 0, 1)`이었다.
RViz 실행과 차량/지도 표시를 확인했다. 검출 박스 실물 정합은 확인하지 못했다.

현재 MORAI 프로세스는 -force-glcore로 실행 중이다. Simulator Player.log에
`This GfxDevice does not support asynchronous readback`과
`Failed to read GPU texture`가 반복된다. GPU 센서 생성 실패가 무수신의 의심 원인이며,
렌더링 모드를 바꾼 뒤 수신 여부를 비교해야 원인 확정이 가능하다.

Unity의 [GPU readback 지원 조회](https://docs.unity3d.com/2020.3/Documentation/ScriptReference/SystemInfo-supportsAsyncGPUReadback.html)는
플랫폼/API 지원 여부를 별도로 제공한다. 이 문서가 MORAI의 현재 오류 원인을 확정하는 증거는 아니다.
시뮬레이션 상태가 초기화되는 재실행은 사용자 확인을 요청한 상태다.
실제 UDP 수신, 검출 결과, 물리 정합과 MORAI closed-loop 성공으로 보고하지 않는다.
