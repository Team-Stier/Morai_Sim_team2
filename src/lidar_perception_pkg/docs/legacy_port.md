# 기존 장애물 검출부 이식

[파일] 원본은 제공된 catkin_ws_real/src/lidar/lidar/object_detector다.
object_detector.cpp의 ROI → VoxelGrid → DBSCAN → AABB를 재사용했다.
dbscan.h를 가져오고 표준 헤더를 명시했으며 이미 처리한 이웃을 새 군집에
다시 삽입하는 경로를 방지했다. 최소 이웃 수는 검색 결과의 자기 자신을 포함한다.

config/detector.yaml은 cfg/config.cfg 초기값을 옮긴 개발 비교 기준이다.
minClusterSize=0은 비어 있지 않은 군집 허용이라는 같은 의미의 1로 정규화했다.
ddd.cfg는 별도 저장 스냅샷이며 launch 참조가 확인되지 않아 기본값으로 채택하지 않았다.
이 값은 MORAI에 최적화된 ROI/지면 높이/군집 파라미터가 아니다.

카메라 투영, section에 따른 미션 ROI, 차선 처리와 차량 제어는 제외했다.
지면 분리·빈 공간·occupancy·속도 추정은 구현하지 않았다. z ROI를 지면 분리로
간주하면 안 된다. 검출된 box는 관측된 점의 범위이며 실제 물체 전체 크기를
보장하지 않는다. confidence=-1은 신뢰도 미보정이다.

## 실행

Ubuntu 20.04 / ROS Noetic에서 저장소 루트 기준:

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
PYTHONNOUSERSITE=1 catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
roslaunch lidar_perception_pkg lidar_perception_pkg.launch
```

입력은 승인된 /molit/sensors/lidar/points와 /molit/sensors/lidar/status만 사용한다.
이 launch는 UDP·TF를 켜지 않으며 system bringup의 비활성 채널도 변경하지 않는다.
MORAI 브리지 활성화는 해당 패키지의 live 검증 절차를 따른다.
RViz Fixed Frame=lidar_link에서 /lidar_perception_node/filtered_points를 확인할 수 있다.
객체는 /molit/perception/lidar/observations로 원본 scan stamp와 함께 발행한다.

## 동작과 제한

- 정상 ROI 결과가 비면 objects_valid=true와 빈 객체 목록을 발행한다.
- 빈 raw scan/전부 NaN/잘못된 layout/transport 실패는 invalid 결과를 발행한다.
- 잘못된 frame, 0·미래·중복·역행 stamp는 관측 발행 없이 상태 오류로 보고한다.
- 고정 100개 배열을 제거해 군집 수에 따른 배열 초과를 방지한다.
- 개발 수신 watchdog은 중앙 lidar_runtime.yaml의 기존 1초 기준을 사용한다.
- 처리 완료로 원본 stamp를 바꾸지 않는다. 과거 결과를 재발행하지 않는다.
- 축·loadout·freshness 검증 전이므로 ready=false, stop_required=true다.
- 센서 프로필 해시는 보정 근거 식별용이며 실제 장착 승인으로 사용하지 않는다.

## 검증

```bash
python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py --check
PYTHONNOUSERSITE=1 catkin_make run_tests
catkin_test_results --all build/test_results
PYTHONNOUSERSITE=1 catkin_make install -DPYTHON_EXECUTABLE=/usr/bin/python3
```

GTest는 두 객체 분리, 크기, ROI, NaN, 노이즈, 100개 초과, 빈 결과와
잘못된 파라미터를 검사한다. rostest는 합성 PointCloud2를 공개 입력으로 보내
객체 출력·원본 시각·빈 검출·입력 오류·watchdog을 검사한다.
공유 메시지 테스트는 중앙 wire 정의, ROS1 직렬화와 소비자 거부 규칙을 검사한다.

[미확정] 작성 PC에는 ROS/WSL 런타임이 없다. catkin/PCL 빌드와 GTest/rostest,
MORAI 점군·폐루프 주행은 여기서 실행되지 않았다. 로컬 검사와 실제 실행
증거는 구분하며, ROS 호스트에서 위 명령을 실행해야 한다.
