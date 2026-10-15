# Camera3 신호등 YOLO 실행

`./run.sh`는 Camera3 UDP 수신과 YOLO를 기본으로 함께 시작한다.
`./run.sh camera3_yolo:=false`로 끌 수 있다. 학습은 실행하지 않는다.

MORAI Sensor 프리셋 `camera3signal_yolo`를 Load하고 Camera3 UDP를
Connect한다. 프리셋 파일은 `morai_interface_pkg/config/camera3_signal_sensor.json`에도
저장돼 있다. 기존 실행 중인 시뮬레이터의 설정을 코드가 자동 조작하지 않는다.

- Camera3: RGB, 위치 (1.8,0,1.3), pitch 320° (= -40°, 2026-10-15 13:53 저장 설정), FOV 90°
- 해상도 1920×1440, 최대 30 Hz, Bounding Box 송출 해제
- 로컬 UDP: host 127.0.0.1:9296 → destination 127.0.0.1:9297
- 원본 ROS 영상: `/molit/sensors/camera/signal/image/compressed`

카메라만 단독 실행하려면:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch system_bringup_pkg camera3_signal.launch
```

기본 실행은 `/molit/route/context`의 유효 진행도가 있어야 추론한다.
중앙 `signal_observation_policy.yaml`의 1·2·3·5번 정지선 전방 30m부터
정지선까지 활성화하고 4번 우회전은 제외한다. 거리는 전역경로 XY 누적 거리이며
원본 TXT hash가 일치해야 한다. 정지선 통과·경로 만료(0.5초)·시각 역행 시
대기·진행 중 결과를 폐기한다. 공개 HSV 관측은 World Model에서 지도 head에 대응하고 Planner가 정지·재출발을 판단한다.
경로 정보 없이 영상 검출만 확인하는 명시적 진단 실행은
`roslaunch system_bringup_pkg camera3_signal.launch signal_route_gate:=false`다.

박스 영상은 별도 터미널에서 다음으로 볼 수 있다:

```bash
rosrun rqt_image_view rqt_image_view /camera_perception_node/signal_image
```

모델은 기존 6클래스 PIDNet과 별개인 YOLO11n이며 3등식·4등식 신호등
하우징 2클래스다. checkpoint 원본은
`/home/paik/morai-artifacts/camera3-route-signal-expansion-20261014/training/camera3_route_signal_first/weights/best.pt`,
SHA256 `e2a5e1569913fbb7a16db4efc3e958875ab2abf9c862575f6e753ea40b223b17`.
로컬 실행 사본은 workspace `weights/camera3_route_signal.pt`다.

기본 Python은 `~/venvs/morai-yolo/bin/python`이다. 통합 실행에서는 카메라 노드 하나가
이 환경에서 PIDNet과 YOLO worker를 함께 실행한다. 이 환경의 Ultralytics와
ROS Python 의존성은 `config/signal_requirements.txt`에 기록했다.
다른 PC에서는 해당 환경을 준비하고 `signal_python:=... signal_weights:=...`를
`run.sh`에 전달한다. 모델은 자동 다운로드하지 않는다.

`~signal_detections` JSON과 `~signal_image`는 하우징 후보 진단 출력이다.
원본 header/stamp를 보존하고 가장 최신 대기 영상 1장만 처리한다.
이미지의 3/4등식 candidate 표시는 YOLO의 하우징 분류이며,
`HSV 3:... / 4:...`는 같은 박스를 두 layout으로 판독한 가설이다.
공개 배열에는 같은 bbox·confidence·원본 stamp로 layout3/4를 각각 발행한다.
private JSON의 `lamp_hypotheses`에 두 판독을 싣고, 기존 `lamp_mask/lamp_valid/lamp_reason`은
YOLO 분류에 해당하는 진단값만 유지한다. 지도 선택 결과나 출발 허가가 아니다.
검출 후보가 다른 차선의 신호일 수 있으므로 World Model이 지도 대상 ID를 선택한다.
Planner 연결은 중앙 HSV 신호 계약을 따르며 private 후보를 직접 제어에 사용하지 않는다.

중앙 공개 입력과 발행하지 않는 TF의 근거는
[Camera3 입력 계약](../../ros_architecture_pkg/docs/camera3_signal_ingress.md)을 따른다.
현재 raw ID3는 개발용 신호 카메라로 사용한다. 본선 고정 Right Camera를
유지하려면 자유 카메라 배치/ID를 별도로 맞추고 센서 제출을 검토해야 한다.

공개 연결 상세: `ros_architecture_pkg/docs/traffic_signal_hsv.md`.
HSV 파라미터는 camera_perception_pkg/config/signal_hsv.yaml, 지도 대응·허용 확인은
world_model_pkg/config/world_model.yaml, 제동 여유는 path_planning_pkg/config/frenet_planner.yaml을 따른다.
