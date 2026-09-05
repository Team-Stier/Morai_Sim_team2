# camera_perception_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- 차량·보행자·장애물 등 영상 객체 관측
- 신호 상태, 차선·실선·중앙선과 주행 가능 영역 관측
- 카메라별 측정 timestamp, calibration ID, confidence와 입력 freshness 제공
- Sunny/Foggy 및 11시·13시·15시 변화에 대한 모델·전처리 검증

## 담당하지 않는 범위

- 객체의 최종 전역좌표 변환과 cross-sensor tracking
- HD Map, 행동 결정, trajectory 생성, 차량 제어
- 시뮬레이터 UDP 직접 수신

## 대회 규정상 유의사항

- 카메라는 최대 4대, 최대 30 Hz이며 고정 3대의 위치·자세·FOV는 변경할 수 없다.
- 현재 제공 설정의 20 Hz는 파일 설정이지 공식 고정 주기가 아니다.
- Ground Truth와 2D/3D Bounding Box를 사용할 수 없다.
- 허용 UDP에 신호등 정답 전용 채널이 없으므로 sample scene의 신호 상태를 런타임 인식 대신 사용하지 않는다.

MORAI SIM: Drive 24.R2의 [일반 ROS 인터페이스](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/ros-2)에는
`/GetTrafficLightStatus`가 문서화되어 있지만, 대회 외부 통신 allowlist에는 해당
신호등 정답 채널이 없다. 따라서 대회 runtime에서는 이 topic/service, V2I 또는
sample scene의 `tlList`를 구독하지 않는다. 현재 신호 상태는 알 수 없는 상태로
취급하며, 향후 전방 카메라의 timestamped 신호 인식과 정적 HD Map의 신호 ID·정지선
association을 World Model에서 결합하는 중앙 계약이 승인된 뒤에만 Planner 입력으로
사용한다.

## 논리 입출력

- 입력: 정규화된 카메라 프레임과 중앙에서 승인한 calibration 정보
- 출력: 센서 관측 좌표의 timestamped detection, lane/signal/free-space 관측과 품질 상태

World Model이 관측 시각의 pose를 사용해 좌표를 통합하므로 이 패키지는 최신 Localization pose로 검출 결과를 임의 투영하지 않는다.

## 디렉터리

- `config/`: 모델·전처리·threshold 등 패키지 로컬 파라미터
- `docs/`: 데이터셋, calibration, 모델 카드와 평가 근거
- `launch/`: Camera Perception 단독 실행
- `src/`: inference, preprocessing와 observation 변환 구현
