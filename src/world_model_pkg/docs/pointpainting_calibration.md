# PointPainting 카메라 보정·RViz 검증 — 2026-10-08

MORAI의 활성 Camera-1이 `Ground Truth: Instance`로 설정되어 있었다.
UI에서 연결을 끊고 `None`(RGB)으로 수정한 후 기존 9290→9291 UDP를 다시 연결했다.
MORAI가 `SensorInfo_2023_Hyundai_Ioniq5.json`에 `cameraType=0`을 자동 저장했다.
Instance 입력으로 수신한 초기 probe는 보정·인식 검증에서 제외했다.
`참고파일들/` 원본과 다른 센서 설정은 변경하지 않았다.

## 사용 값과 확인 범위

활성 표준 Camera UI의 **Horizontal FOV=90°**, 해상도 **1280×720**을 확인했다.
따라서 `fx=fy=640`, `cx=640`, `cy=360`, distortion=0을 중앙 계약에 명시했다.
JSON의 `focalLengthpixel=320`은 Physical Camera용 비활성 설정이다.
표준 Camera와 Physical Camera 설정은 [MORAI 공식 매뉴얼](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/sensors)을 따른다.
전방 mount는 `[1.9, 0, 1.2] m`, RPY `[0, +2, 0]°`, LiDAR는 `[2, 0, 1.5] m`, RPY=0이다.
중앙 mount 및 REP-103 optical 행렬로 상대 변환을 계산한다.
공식 Velodyne ROS driver의 출력은 [x-forward/y-left/z-up](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/sensor-coordinate-system)이다.

RGB 영상에서 기둥·신호등 가로대 9곳의 중심선을 수동 표시하고,
해당 실측 LiDAR return 25개에서 영상 중심선까지의 수직 거리를 계산했다.
대응점은 XYZ cluster로 지정하며, 오차 기준으로 골라내지 않는다.
최적화로 이 장면에 맞춘 파라미터를 만들지 않고 설정값과 두 잘못된 후보를 비교했다.

| 후보 | 중앙값 | p95 | 최대 |
|---|---:|---:|---:|
| fx=fy=640, pitch +2° | 1.11 px | 2.24 px | 3.58 px |
| fx=fy=320 | 30.63 px | 290.82 px | 296.27 px |
| pitch -2° | 44.88 px | 46.36 px | 46.41 px |

동일 정지 장면의 다음 2개 scan에서도 같은 결과였다. simulator의 정적 구조물이므로
XYZ는 반복되며, 독립 장면이나 전체 6DoF calibration 검증으로 해석하지 않는다.
원본 영상/점군 hash, 측정시각, 수동 대응점은
[검증 fixture](evidence/pointpainting_calibration_20261008.json)에 보존했다.
단위 테스트가 실측 대응점의 p95<3px, 최대<4px를 검사한다.
Hidden simulator 위치·객체 ID·Bounding Box를 투영 계산에 사용하지 않았다.

카메라 TF 발행은 계속 비활성이다. `calibration_verified=false`는 전체 6DoF와
동적 정합 미검증을 뜻하며 중앙 계약에 정지 장면 확인 범위를 별도로 기록했다.
RViz는 `lidar_link`에서 원본 점을 표시하고 이미 승인된 GPS/IMU/LiDAR static TF만 사용한다.

## 실제 출력과 조회

RGB+LiDAR+PIDNet+RViz를 동시에 실행하고 15.02초 동안 painted cloud 93개를 수신했다.
초기 연결 경계의 2개를 제외한 91개를 원본 scan/score와 대조했다.
모든 원본 field, 점 수·순서, LiDAR/frame 및 영상 stamp, 6채널 확률, OpenCV 독립 투영,
class/confidence/RGB bit packing 검사 오류는 **0개**였다.
출력 6.19Hz, 수신 age 중앙값 172.6ms/p95 213.4ms, 최대 pairing 차이 49.64ms였다.
관측 점 수 중앙값은 3,543/14,147개다. 실행 부하에 따라 주기는 달라진다.

검사한 점의 class_id는 background 325,079개, animal 90개, 미관측 962,268개였다.
현재 RGB 장면에서 동물이 확인되지 않아 이 소수 animal 예측은 인식 정확도 근거로 사용하지 않는다.
객체 class 정확도, 3D detector, 이동 중 정합, occlusion/deskew는 검증 범위에 없다.

RViz Select로 점을 선택하면 `score_0`~`score_5`, `painted`, `class_id`, `confidence`,
영상 stamp와 원본 intensity/ring/time을 Selection에서 확인한다.
`rgb`는 표준 PCL FLOAT32 packed RGB bitcast다. 숫자 float로 색상을 계산하지 않는다.
UINT32 RGB는 RViz Selection이 잘못 표시하여 FLOAT32 표준 형식으로 수정하고 실제 UI로 확인했다.
RGB Image plugin의 자동 mouse-click 출력만 RViz private namespace로 보낸다.
공개 입력 topic이나 node는 remap하지 않는다.

실측 출력·화면·CSV는 `/home/paik/morai-artifacts/pointpainting-verification/`의
`live-rgb-rviz.json`, `rgb-painted-sample.npz`, `sample-point-values.csv`,
`rviz-values-final.png`, `calibration-checked-0.png`에 있다.

## 이 PC에서 실행

이 PC는 sudo 비밀번호가 필요하여 표준 `compressed_image_transport` 및 `libturbojpeg`
배포 deb를 사용자 경로에 압축 해제했다. 정상 시스템 설치에서는
`ros-noetic-compressed-image-transport` 의존성을 설치하면 된다.

```bash
source /home/paik/Morai_Sim_team2/devel/setup.bash
source /home/paik/morai-artifacts/pointpainting-image-transport/setup.bash
roslaunch system_bringup_pkg pointpainting.launch \
  python:=/home/paik/venvs/morai-pidnet/bin/python \
  weights:=/home/paik/morai-artifacts/pidnet-training/output/Morai6/pidnet_s_morai6/best.pt \
  rviz:=true
```

이미 같은 노드가 실행 중이면 중복 실행하지 않는다. 카메라 bridge와 수동 격리 LiDAR probe는
별도로 연결한다. LiDAR UDP 채널의 system bringup 자동 활성화 금지는 유지한다.
이번 변경은 로컬 commit만 하며 push하지 않는다.

2026-10-09에 PointPainting의 구현·보정 커밋만 로컬 `paik`에 통합했다.
위 명령은 현재 `paik` 작업 폴더의 빌드 결과를 사용한다. 이 PC의 `ros_numpy`는
`/home/paik/morai-artifacts/ros-numpy-catkin`에서 별도로 빌드했으며, 현재 작업
폴더의 `devel/setup.bash`가 해당 의존성 환경을 함께 불러온다.
재빌드할 때는 아래 순서로 의존성 환경을 먼저 등록한다.

```bash
source /opt/ros/noetic/setup.bash
source /home/paik/morai-artifacts/ros-numpy-catkin/devel/setup.bash
cd /home/paik/Morai_Sim_team2
PYTHONNOUSERSITE=1 catkin_make -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

통합 후 현재 작업 폴더에서 catkin 빌드, Camera/World Model/Visualization/중앙 계약의
패키지 테스트, PointPainting ROS transport 테스트, static TF 계약 테스트 4개와
다이어그램 동기화 검사를 통과했다. launch XML·YAML·manifest 및 전용 launch의
파라미터 확장도 확인했다. 실제 `best.pt`를 GPU에 엄격하게 로드하고 저장된 RGB 영상의
720×1280×6 확률 출력과 픽셀별 확률 합을 확인했다. 기존 경로계획·Localization·HD Map의
커밋된 코드는 통합 전후 동일하며, 사용자 미커밋 파일은 SHA-256으로 보존을 확인했다.
검사 로그는 `/home/paik/morai-artifacts/pointpainting-paik-integration/`에 있다.
이번 통합에서는 라이브 센서 노드를 재시작하거나 MORAI 주행을 수행하지 않았다.

## 최종 빌드·계약 검사

- Catkin 의존 패키지 포함 빌드 성공. Camera/World Model/Visualization/System Bringup/중앙 계약의
  catkin 결과 집계 **129 tests, 0 errors, 0 failures**.
- 기존 static TF gate 단위 검사 4개 추가 통과. 단일 parent, cycle 부재, 비활성 카메라 TF 발행 금지를 유지한다.
- 전체 launch/XML/YAML parsing, `roslaunch --dump-params`, 공개 producer/consumer 검사,
  Mermaid/SVG/PNG 18개 생성 및 `generate_interface_diagrams.py --check` 통과.
- 기존 Visualization fixture의 첫 clock 메시지는 latch하여 늦게 연결된 테스트 구독자도 받도록 했다.
  LiDAR 표시 fixture는 모든 transport 연결을 기다리고, Localization attitude가 없는 이 fixture에만
  `leveling_enabled=false`를 명시했다. 실제 detector의 leveling 기본값은 바꾸지 않았다.
- 직접 작성한 Python runtime 합계 224줄. PIDNet, Velodyne driver, ros_numpy, message_filters,
  RViz 및 compressed_image_transport를 재사용한다.

빌드·전체 검사 로그는 위 artifact 경로의 `build-final.log`, `tests-complete.log`,
`diagrams-final.log`에 있다. 이 검증은 PointPainting 입력·출력과 표시를 다루며
MORAI 주행 closed-loop나 Planner/Controller 성능 검증은 수행하지 않았다.
