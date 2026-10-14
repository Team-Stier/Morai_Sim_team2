# MORAI PointPainting 개발 경로

Camera Perception이 발행한 H×W×6 확률맵과 raw LiDAR를
ROS `ApproximateTimeSynchronizer`로 묶고, NumPy에서
`camera <- lidar` 변환·pinhole projection·픽셀 조회를 수행한다.
[PointPainting 원 논문](https://arxiv.org/abs/1911.10150)의 score-decoration 단계다.
학습된 3D detector, 객체 class association과 Planner 연결은 이 변경의 범위에 없다.

중앙 원본은 [pointpainting.yaml](../../ros_architecture_pkg/config/messages/pointpainting.yaml),
[interface_contract.yaml](../../ros_architecture_pkg/config/interface_contract.yaml),
TF·timestamp 계약이다. `camera_perception_node`가 semantic score를 생성하고
`world_model_pkg/pointpainting_node`가 두 센서를 융합한다. 기존 `world_model_node`는
독립적으로 실행 가능하며 painted output을 현재 소비하지 않는다.

원본 LiDAR의 모든 field 값, 점 개수·순서·header stamp/frame을 유지하고 다음 field를 추가한다.
`header.seq`는 rospy가 출력 publisher의 순번으로 부여한다.

| Field | Type | 의미 |
|---|---|---|
| `score_0` … `score_5` | FLOAT32 | background, person, animal, box, bollard, barrier 확률 |
| `painted` | UINT8 | 유한·전방·영상 안쪽 투영점이면 1 |
| `image_stamp_sec`, `image_stamp_nsec` | UINT32 | 사용한 카메라 영상의 원본 시각 |
| `class_id` | UINT8 | 관측이면 argmax 0~5, 미관측이면 255 |
| `confidence` | FLOAT32 | 관측이면 최대 확률, 미관측이면 0 |
| `rgb` | FLOAT32 packed RGB | 중앙 palette 색의 PCL bitcast; RViz RGB8 표시 |

화면 밖·카메라 뒤·NaN 좌표인 점도 보존하고 `painted=0`, 확률은 모두 0이다.
이를 배경 예측으로 해석하면 안 된다. 출력은 little endian, unorganized cloud다.
이전 ring/intensity/time 등 field 값도 보존한다.

메시지 field/dtype 변환과 출력은 MIT 라이선스의
[ros_numpy](https://github.com/eric-wieser/ros_numpy), commit
`74879737c8648f48adb507a5bdf4e51c0d194124`를 재사용한다.
이 라이브러리는 package.xml의 runtime dependency다. 설치가 없으면 catkin workspace
`src/ros_numpy`에 upstream checkout을 두고 위 commit을 사용하면 된다.
추가한 짧은 strided reader는 upstream 변환의 row padding·big endian 제한을 처리한다.
다른 PointPainting 저장소의 KITTI calibration·Cityscapes 모델은 MORAI에 맞지 않아 포함하지 않았다.

## 좌표·시간의 현재 범위

카메라 mount·optical TF의 `publish_enabled=false`는 유지한다.
중앙 sensor extrinsics의 **candidate** 행렬을 읽어 개발용 local projection을 계산한다.
Front 1280×720, horizontal FOV 90°로 fx=fy=640, cx=640, cy=360을 유도한다.
2026-10-08 활성 표준 Camera UI의 Horizontal FOV=90을 확인하고 실측 구조물 투영을 검사했다.
JSON의 focalLengthpixel=320은 Physical Camera용 비활성 설정이다.
현재 central intrinsic matrix는 기준 해상도에 명시되며 다른 해상도에서는 축별 비율로 scale한다.
전체 6DoF와 동적 정합은 미검증으로 `calibration_verified=false`를 유지한다.
[보정·RViz 실제 검증](pointpainting_calibration.md)의 범위를 따른다.

sync slop 0.05초, pair 최대 age 0.5초, sync queue 4가 개발 기본값이다.
0 stamp, stale/future·frame 불일치·같은 clock epoch 안의 역행은 버린다.
ROS clock reset에서는 synchronizer를 비우고 출력 순서 상태도 초기화한다.
입력 단절 때 새 출력을 만들거나 이전 쌍을 재발행하지 않는다.
전체 scan deskew, ego motion 보상과 occlusion filtering은 구현하지 않았다.
출력에는 driving consumer가 없고 제어·Safety 정책을 바꾸지 않는다.

## 검증

검증 기록은 아래에 실제 수행 결과를 기입한다.

- catkin build: camera/world-model/system bringup과 의존 패키지 빌드.
- 단위 테스트: 앞·뒤·화면 밖·NaN 점, 중앙 mount 투영, 6채널 ROS Image 왕복,
  big endian·row padding 입력, 원본 field·두 source stamp 보존.
- producer/consumer 계약: 클래스 순서, ROS type·소유 패키지, frame·timestamp 등록.
- ROS transport test: 실제 synchronizer callback·painted output·stale pair 무출력.
- 학습 가중치: strict load와 full-resolution softmax 합 확인. 학습 Morai6 loader의
  같은 입력에 대한 확률맵과 최종 adapter 출력의 최대 차이 0, argmax 일치 확인.
- 실제 MORAI bag 및 live 검증 결과는 아래와 같다.

## 2026-10-08 수행 결과

아래는 초기 실행 결과다. 최신 보정·RViz 추가 검증에서 저장 profile의 GPS/IMU 범위를
재확인하고 hash 근거를 갱신했으며, 최종 catkin 집계 129개 모두 통과했다.
[최신 검증 기록](pointpainting_calibration.md)을 기준으로 현재 상태를 확인한다.

- 16개 catkin 패키지(의존성 포함) 빌드 성공. XML/YAML parsing, 중앙 계약 및
  Mermaid/SVG/PNG hash 검사 통과. Camera/World Model 단위·계약·ROS transport 테스트 통과.
- 확장 검사 88개 중 87개 통과. 남은 1개는
  `test_observed_user_confirmed_profile_matches_its_scope`의 저장된 sensor profile
  SHA 불일치다. 원래 `/home/paik/Morai_Sim_team2`에서도 동일 실패를 재현했다.
  현재 파일 `d5c8d2d3…`, 계약 `3312ca969…`이며 이번 작업에서 profile/TF를 변경하지 않았다.
- 실제 bag `2026-10-06-00-13-29.bag`의 +5초부터 10초 구간을 두 번 재생했다.
  Camera 400장/LiDAR 180 scan 입력에서 painted output 170개를 검증했다.
  original x/y/z/intensity/ring/time 값·점 개수, LiDAR stamp/frame, camera stamp,
  확률 범위·합, unobserved zero, 50ms pairing 검사 오류 0개.
  ROS clock rewind 후 출력 재개, input stop 후 무출력을 확인했다.
- 격리된 ROS master 11318에서 실제 MORAI Camera UDP와 VLP16 ingress를 수신했다.
  최종 전처리 버전에서 control sender 없이 15.01초 동안 Camera 300장, LiDAR 121 scan,
  score image 173장, painted cloud 116개를 확인했다.
  측정 rates: Camera 19.98Hz, LiDAR 8.06Hz, score 11.52Hz, painted 7.73Hz.
  subscriber 기준 output age 중앙값 128.8ms, p95 174.3ms. 확률·pairing 검사 오류 0개.
  이 수치는 simulator와 verifier도 실행 중인 RTX 3060 PC에서 측정했으며 20Hz inference를 보장하지 않는다.
- 실제 bag 3개 frame의 원본 영상·segmentation·projected point overlay와
  decorated point NumPy sample을 저장했다. 라이브 probe의 painted argmax는 모두 배경이었다.
  객체가 있는 장면에서 객체별 3D 정확도 평가는 하지 않았다.
  이후 정지 장면의 extrinsic/intrinsic 투영과 RGB 복구·RViz 표시를 추가 검증했다.
  occlusion과 이동 차량의 시간 오차에 대한 정량 검증은 남아 있다.

로컬 상세 증거: `/home/paik/morai-artifacts/pointpainting-verification/`의
`replay.json`, `live-final.json`, `model_equivalence.json`, `offline_samples.json`, `projection_overlay.png`,
`painted_sample.npz`. sensor probe와 임시 ROS master는 검증 후 종료했다.

## PIDNet foreground 객체 처리 (2026-10-15)

`frenet_all.launch`(`./run.sh`)와 `competition.launch`는 기본으로
`pidnet_enabled:=true`를 사용한다. 전방 RGB → PIDNet 6채널 확률 →
PointPainting → World Model 내부 장애물 군집/추적 → 기존 Planner 순서다.
이 모드에서는 `lidar_perception_node`를 실행하지 않고 raw-only 검출로 되돌아가지 않는다.
World Model의 내부 `/molit/internal/world_model/pidnet_points`는 다른 패키지가 구독하지 않는다.

중앙 `config/messages/pointpainting.yaml`의 foreground 정책으로
person/animal/box/bollard/barrier argmax 및 확률 0.6 이상인 실제 LiDAR 반환점만 선택한다.
배경, 카메라 시야 밖, NaN과 낮은 확률 점은 제외한다. 모델에 차량/신호 클래스는 없다.
Softmax는 보정된 객체 confidence가 아니므로 추적 객체 confidence는 -1을 유지한다.
원본 LiDAR 좌표·field·측정시각과 카메라 측정시각을 보존한다.
객체 군집은 World Model의 `config/world_model.yaml`에서 클래스별 0.4 m voxel
인접성, 최소 3점으로 설정한다. 배경 지면 기반 기존 LiDAR 검출은 사용하지 않는다.

RViz는 `pidnet_vehicle.rviz`로 PIDNet 장애물 점만 RGB로 표시한다.
기존 LiDAR/World Model 장애물 marker 표시는 꺼지고 지도·차량·trajectory는 유지한다.
입력 단절 시 마지막 점군을 재발행하지 않으며 RViz 점 표시의 유지 시간은 0.5초다.
World Model은 측정시각과 상태 watchdog으로 stale 결과를 거부한다.
객체가 없는 결과는 free-space 증거가 아니다. 개발 calibration/freshness와
planner_ready 검증 제한은 유지한다.

현재 PC의 가중치와 Python 기본값은 기존 검증된 경로다. 다른 PC에서는 인자로 지정한다.

```bash
./run.sh pidnet_weights:=/absolute/path/best.pt pidnet_python:=/absolute/path/python
./run.sh send_to_morai:=false rviz:=false
# 기존 LiDAR 검출을 명시적으로 사용하는 비교 실행:
./run.sh pidnet_enabled:=false
```

두 launch를 중복 실행하지 않는다. 현재 작업은 별도 worktree에서 수행했으며
원래 작업 폴더의 실행 중 노드를 재시작하지 않았다. 새 코드를 실행할 때 기존
run.sh 터미널에서 Ctrl+C 후 새 worktree의 run.sh를 사용한다.

검증: catkin build, 합성 ROS producer/consumer 통합에서 foreground 선별·map 추적·
원본 stamp 보존·배경/후방 제외·카메라 단절 및 raw-only fallback 부재를 확인했다.
실제 best.pt GPU 추론은 현재 live 입력 3쌍으로 확인했다. 첫 추론 워밍업 후
추론은 약 58~70 ms, 전체 처리 약 123~143 ms였다. 당시 foreground는 0점이었으며
실제 장애물 검출 성능, 동적 calibration과 MORAI closed-loop 주행은 검증하지 않았다.
근거는 `/home/paik/morai-artifacts/pidnet-detection-live-probe.json`에 있다.
