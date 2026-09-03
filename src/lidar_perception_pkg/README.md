# lidar_perception_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- point cloud 유효성 검사, ROI와 지면 분리
- 3D 장애물·객체 군집화, 크기·상대 위치·속도 관측
- free-space와 occupancy 관측
- 측정 timestamp, calibration ID, confidence와 입력 freshness 제공

## 담당하지 않는 범위

- Camera와의 최종 융합, 전역 객체 추적과 planner-ready world model
- 행동 결정, trajectory와 차량 명령
- 시뮬레이터 UDP 직접 수신

## 대회 규정상 유의사항

- 3D LiDAR는 최대 1대이며 `VLP16`, Intensity 방식만 허용된다.
- 회전율은 최대 15 Hz이고 공지 권장은 10 Hz 이하이다.
- 현재 센서 설정 파일에는 LiDAR가 없으므로 장착 위치·회전율·포트를 추측하지 않는다.
- sample scene의 객체 목록이나 Ground Truth를 검출 결과로 사용하지 않는다.

## 논리 입출력

- 입력: 정규화된 point cloud와 중앙에서 승인한 calibration 정보
- 출력: 센서 관측 좌표의 timestamped 3D object/obstacle/free-space와 품질 상태

오래된 장애물을 현재 관측처럼 유지하지 않고, sparse VLP16 환경에서의 miss와 uncertainty를 명시한다.

## 디렉터리

- `config/`: ROI, filter, clustering과 모델 로컬 파라미터
- `docs/`: calibration, 데이터 특성, 알고리즘과 평가 근거
- `launch/`: LiDAR Perception 단독 실행
- `src/`: point cloud 처리와 observation 생성 구현
