# visualization_pkg

한 RViz 화면에서 지도, 차량 위치, 센서 관측과 주행 경로를 확인하기 위한
읽기 전용 시각화 패키지다. 현재는 **패키지 골격만 생성**했으며 RViz 실행,
표시 설정, 데이터 변환 노드는 아직 구현하지 않았다.

## 담당 범위

- `config/`: 단일 RViz 화면의 display·색상·카메라 설정
- `src/`: 향후 필요한 시각화 전용 변환 도구
- `launch/`: 시각화 패키지의 단독 실행
- `docs/`: 표시 대상·좌표계·검증 기록

전체 시스템과 RViz를 함께 시작하는 실행 조합은 `system_bringup_pkg`가 소유한다.
제어 명령, 차량 위치 추정, 동적 객체 추적과 지도 생성은 해당 소유 패키지가 담당한다.
RViz의 초기 위치 지정·목표 전송처럼 주행 입력을 발행하는 도구는 이 골격에 포함하지 않는다.

## 공개 ROS 입출력

중앙 [계약](../ros_architecture_pkg/config/interface_contract.yaml)에 현재
런타임 I/O가 없는 패키지로 등록했다. 공개 node, 입력 topic, 출력 topic은 모두 없다.
향후 표시 입력과 시각화 변환이 필요하면 중앙 계약의 producer/consumer와
frame·timestamp를 먼저 정하고 구현한다. 빈 목록은 임의 인터페이스 생성을 허용하지 않는다.

![Visualization 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** 없음

| 구분 | Topic | Type |
|---|---|---|

현재 launch는 placeholder로 실행해도 RViz 창을 열지 않는다.

## 후속 연결 시 확인할 사항

- 승인된 공개 데이터만 구독하며 다른 패키지의 내부 topic을 직접 읽지 않는다.
- 지도·차량·센서가 동일 화면에 정합되려면 중앙 TF와 timestamp 검증이 먼저 필요하다.
- 미검증 TF를 시각화 편의를 위해 임의로 발행하지 않는다.
- 현재 HD Map은 오프라인 파일 생성 단계이고 Localization 공개 ROS adapter도 비활성이다.
  실제 지도·위치 publisher가 준비된 범위부터 표시한다.
