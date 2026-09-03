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

## 논리 입출력

- 입력: 정규화된 카메라 프레임과 중앙에서 승인한 calibration 정보
- 출력: 센서 관측 좌표의 timestamped detection, lane/signal/free-space 관측과 품질 상태

World Model이 관측 시각의 pose를 사용해 좌표를 통합하므로 이 패키지는 최신 Localization pose로 검출 결과를 임의 투영하지 않는다.

Camera mount/optical frame은 중앙 [`TF 계약`](../ros_architecture_pkg/config/tf/frame_contract.yaml)을 따르고, MORAI의 `Camera-1/2/3` 문자열을 새 중앙 frame 이름으로 사용하지 않는다. Detection은 [`Timestamp 계약`](../ros_architecture_pkg/config/timestamp/timestamp_contract.yaml)에 따라 원본 영상의 측정시각을 유지하며 inference 완료 시각으로 덮어쓰지 않는다.

## 디렉터리

- `config/`: 모델·전처리·threshold 등 패키지 로컬 파라미터
- `docs/`: 데이터셋, calibration, 모델 카드와 평가 근거
- `launch/`: Camera Perception 단독 실행
- `src/`: inference, preprocessing와 observation 변환 구현
