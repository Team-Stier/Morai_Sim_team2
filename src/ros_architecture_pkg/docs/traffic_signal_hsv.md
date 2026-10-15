# HSV 신호 판독·World Model 선택·Planner 정지·RViz 표시

사용자 승인 범위(2026-10-15): YOLO 박스 내부 HSV 판독, 지도 대상 선택,
기존 RViz 신호 표시와 실제 적색 정지/허용 신호 재출발을 연결한다.

공개 변경은 CameraSignalArray/TrafficSignal 타입, Camera signal observations topic,
WorldModel.traffic_signals다. 기존 scene consumer는 같은 message MD5로 재빌드한다.
Camera는 원본 영상 stamp와 3/4구 점등 bitmask를 제공하고, World Model이 source
pose 보간 및 카메라 source extrinsic으로 head ID를 대응한다. 카메라 TF는 발행하지
않는다. 1·2·3·5번의 원본 head 위치·layout·진행 방향은 중앙 관측 정책이 소유한다.
정적 위치는 MGeo metadata이며 phase Ground Truth나 판정 전용 UDP를 사용하지 않는다.

0.5초 만료/시각 역행/Localization reset/다른 route hash/잘린 ROI/모호한 투영은
허용 상태가 아니다. World Model의 source stamp는 LiDAR scene stamp와 독립적이다.
연속 3프레임·0.25초의 허용 상태를 확인하며 동시 head는 공통 점등을 사용한다.
GREEN은 직진, LEFT는 좌회전이며 적색+LEFT는 좌회전만 허용할 수 있다.

Planner는 공개 scene에서만 상태를 읽고 두 플래너·유지 경로에 정지선 감속 곡선을
매 발행 적용한다. 원본 경로 거리의 30m 접근 범위, base_link 정지 목표 직진 8m/좌회전 14m 전,
유효 허용 신호로 직진 8m/좌회전 12m 안에 진입한 이후 확정 진행을 사용한다. 정지 목표는 차량 앞끝
3.845m 및 카메라 시야/제동 여유를 포함하는 개발값이다. RED/YELLOW/UNKNOWN/stale는
정지 유지하며 허용 상태로 바뀌면 원래 trajectory를 재계획한다. accel/brake/steer는
기존 Controller→Safety→morai_interface 경로만 소유한다.

RViz의 기존 World Model marker에서 같은 선택 상태를 색상·텍스트로 표시한다.
위치·방향 물리 보정과 전체 World Model readiness는 여전히 개발 단계다.

## 2026-10-15 실제 실행 검증

일반 `./run.sh rviz:=true test_speed_cap_kph:=15.0`으로 실제 MORAI 카메라,
GPS/IMU/LiDAR 콜백과 Controller→Safety→UDP 명령을 함께 기록했다. 이는
개발용 속도 제한 설정이며 실제 속도가 항상 15km/h 이내라는 검증은 아니다.
원시 증거는 `/home/paik/morai-artifacts/hsv-signal-closed-loop-20261015/`의
`events.jsonl`, `run2/events.jsonl`, `run3/events.jsonl`, `run4/events.jsonl`, 원본·주석 영상과 실행 로그다.

| 대상 | 정지/재출발 관측 | 실행 |
|---|---|---|
| 1 직진 | 적색 정지, base_link s=201.48m(정지선 약 6.98m 전), 녹색 재출발·통과 | 최초 실행 |
| 2 직진 | 적색 정지 s=307.42m(약 8.49m 전), 녹색 재출발·통과 | run2 |
| 3 직진 | 적색 정지 s=429.55m(약 7.25m 전), GREEN+LEFT 중 직진 녹색으로 재출발·통과 | run2 |
| 4 우회전 | 관측 정책 대상 제외 유지 | run2 |
| 5 좌회전 | 기존 위치에서 자동 적색 정지·일반 GREEN 대기. 수동 후진 후 약 15.8m 전 정지 상태에서 RED+LEFT 확인, 자동 재출발·정지선 통과 | run2/run3/run4 |

최초 2번 시험에서는 영상 상단 잘림 이후 UNKNOWN으로 다시 감속하여 정지선에
너무 가깝게 정지했다. 허용 신호를 확인하고 8m 이내에 도달한 경우 교차로 진행을
확정하도록 바꾸고, 긴 계획 계산 이전의 World Model/현재시각을 재사용하지 않도록
수정했다. run2에서 같은 구간 재출발·통과를 재확인했다. 재시험 위치 준비에는
MORAI의 수동 후진을 사용했고 해당 구간은 자동 주행 성공 증거에서 제외한다.
run3은 관측 초기화 보완을 포함한 정지 중 재시작 검증이다. run4는 좌회전
시야 여유를 확대한 최종 코드로 실제 위치를 수동 후진한 뒤 재시험했다.
7m 전 정지 위치에서는 좌회전 화살표를 해상하지 못했고, 후진한 위치에서는
동일 대상의 RED+LEFT가 판독됐다. 차양/관측각 영향이 의심되어 좌회전 정지
목표를 14m, 허용 시 진행 확정을 12m로 분리했다. run4는 이미 정지한 상태에서의
좌회전 허용 재출발 검증이며, 변경된 14m 목표로 접근하여 적색 정지한 연속 시험은
아직 수행하지 않았다. 첫 LEFT 유효 관측 wall=1792039704.921, 실제 전진
v>0.5m/s=1792039705.807, 확정 진행=1792039715.368, s>620m 통과=1792039737.625다.
카메라/World Model source stamp와 실제 EgoState 속도·RouteContext 및 최종
명령을 함께 기록했다. 신호 상태를 UDP나 테스트 메시지로 주입하지 않았다.

RViz 적색 정지 화면은 `run2/rviz-red-stop-5.png`에 있다. run4의 출발 직후
화면은 RViz 지도 로딩 중이므로 LEFT 표시의 GUI 증거로 사용하지 않는다.
RViz GREEN/LEFT MarkerArray 송수신은 run2의 3번 통과 로그로 확인했다.
시험 종료 시 Q handover로 UDP를 일시정지하고 Manual-keyboard에서 제동하여
GUI 0.00km/h를 확인했다. 표시와 센서 파이프라인은 켜둔다.

catkin 전체 빌드, Camera 15·공유 메시지 43·World Model 41·Planner 132·
Visualization 46·중앙 계약 46개 테스트 및 중앙 그림 hash 검사를 통과했다.
World Model/Visualization ROS 수송 시험은 별도 master에서 실행했다.
원래 stock 실행을 막던 `ros_numpy`는 공식 ROS deb에서 사용자 Python site에
설치하여 별도 PYTHONPATH 없이 실행했다. 다른 PC는 기존 manifest의 rosdep
의존성을 설치해야 한다.

이번 결과는 현재 카메라 배치·조명·경로에서의 개발 검증이다. 실제 위치/방향
보정, 전체 World Model readiness, 모든 날씨·속도·반복 주행의 안전성 인증을
대신하지 않는다. 잘린/만료/불일치 관측은 허용 신호로 사용하지 않는다.
