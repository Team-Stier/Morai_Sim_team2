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


## Camera3 pitch 변경과 추가 인식 확인

사용자가 정지선 근처 화면 상단 잘림을 줄이기 위해 Camera3 pitch를 340°에서
320°(-40°)로 변경했다. 2026-10-15 13:53 저장 프로필과 실제 Sensor Editor의
위치 [1.8,0,1.3]·pitch 320°를 확인했다. 저장 프로필, 중앙 extrinsic candidate,
World Model 투영을 함께 동기화했으며 카메라 TF 발행은 계속 비활성화한다.
앞 절의 run1~run4 주행 결과는 변경 전 340° 설정의 증거다.

새 설정의 실제 1920×1440 RGB에서 source stamp=1792040461.651의 4구 하우징을
YOLO confidence=0.9625994, bbox=[897.81,928.92,1052.57,980.19]로 검출했고
HSV RED(mask=1, valid=true)로 판독했다. 증거는 artifact의
`pitch320-preview/observation.json`과 `pitch320-preview/annotated.jpg`다.
해당 주석 영상의 source stamp는 1792040461.741이며 바로 다음 측정이다.
새 pitch의 지도 대응 및 연속 접근·정지·재출발 시험은 기존 결과와 별도로 기록한다.

Camera 15·MORAI Interface 86·중앙 계약 46개 테스트와 catkin 빌드를 통과했다.
13:53 프로필의 GPS/IMU ID, target, 위치·각도·frame·주기를 원본과
대조해 변하지 않았음을 확인했다. 재개 시 호스트 저장 파일이 이전 340°
프로필 hash로 복원되어 현재 GPS/IMU 증거 hash는 이에 맞췄고, 13:53 hash는
과거 관측으로 보존했다. Camera3 전용 원본은 저장소의 320° 프로필이다. 사용자 화면을 방해하지 않도록 시뮬레이터/RViz를 별도 작업공간으로
옮겼다. 수동 위치 준비 및 초기 위치 reset 구간은 자율주행 성공 증거에서 제외한다.


## 재개 시험: 세션 종료로 연속 주행 미완료

`pitch320-resume/`의 첫 준비 시도는 저장 프로필이 340°로 복원되어
중단한 시험이다. 디렉터리 이름 자체를 320° 활성 증거로 사용하지 않는다. 사용자 입력을
방해하지 않도록 Xephyr `:99`에서 MORAI와 RViz를 실행했다. 시험 장면의
Ego 초기 배치는 과거 GPS/IMU 기반 EgoState를 복사한 준비 절차이며 실제 위치
검증이나 알고리즘 입력으로 사용하지 않았다. 원본 sample scene은 수정하지 않았다.

기본 저장 Sensor 프로필을 불러온 후 실제 EgoState 위치
(-44.687, 98.140)m, 속도 약 0m/s를 수신했다. 지도 시작을 기다리는 동안
정차한 차량의 UDP sender를 SIGSTOP으로 보류했다. 접근 주행 전에
`SESSION_NOT_EXIST`가 표시되고 Simulator가 정상 종료 코드 0으로 종료했다.
따라서 이번 기록에는 새 각도의 지도 신호 대응·적색 자동 정지·허용 신호
재출발을 입증하는 연속 주행이 없다. 로그인 화면의 MORAI 런처를 별도 화면에
열었다. 사용자가 제공한 계정으로 로그인 후 전용 320° 프로필을 명시적으로
불러와 다시 시험한다. 세션 오류와 종료의 인과관계는
확정하지 않았다. `validation-result.json`, `simulator.log`, `events.jsonl`에
실패/미완료 근거를 남겼다. 정지된 시뮬레이터를 대상으로 실행하던 ROS도 종료했다.


## 정상 로그인 후 320° 실제 접근·정지

`pitch320-live/`는 정상 런처 로그인 후 전용 `000codex_pitch320_sensor.json`을
불러온 시험이다. `sensor-settings.png`에서 실제 Camera3 위치 [1.8,0,1.3]와
pitch 320°를 확인했다. GPS/IMU 기반 RouteContext s=577.826m에서 정상 sender를
재개하여 실제 속도 최대 3.855m/s로 접근했고, s=596.671m(정지선 13.206m 전)에
자동 정지했다. 정지 후 약 28초간 실제 위치와 속도가 정지 상태를 유지했다.
접근 중 GPS innovation reset 1회가 발생했으므로 단일 속도 0 표본만으로
정지를 판단하지 않았다.

실제 영상에서 278개 후보 관측이 RED/valid였고 confidence 범위는
0.834~0.958이었다. 정지 위치에서 bbox 상단이 약1029px/1440px였고 하우징
잘림은 없었다. World Model은 C1256W000052/B2256W000252에 대응하여 적색을
제공했고 Planner STOP과 RViz RED marker를 기록했다. 선택된 신호는 World
Model이 결정하며 영상의 housing candidate 표시는 선택 결과 자체가 아니다.

좌회전 허용 신호를 기다리던 중 런처와 시뮬레이터가 함께 종료됐다. 이번
정상 로그인 시험 로그에는 SESSION_NOT_EXIST가 확인되지 않았으며 종료 원인을
확정하지 않았다. 320°에서의 좌회전 화살표 판독·재출발·통과는 아직 입증하지
못했다. `validation-result.json`, `events.jsonl`, 원본·주석 영상과 ROS 로그를
함께 확인해야 한다. 이 시험을 신호 주기 전체 또는 모든 대상 검증으로 해석하지 않는다.


## 최종 재시험: 320° 연속 접근·정지·좌회전 출발 통과

정상 로그인 후 분리 화면의 독립 실행 환경에서 `pitch320-retry/`에 새 기록을
남겼다. 같은 320° 전용 프로필을 불러왔고 `sensor-settings.png`에서 실제
Camera3 pitch 320°를 재확인했다. 시작 준비에만 scene load와 정차 중 sender
보류를 사용했으며, 정상 sender 재개 이후 정지선 통과까지 수동 입력이나
가짜 관측·신호 상태 주입을 사용하지 않았다. 규정 허용 센서와 기존
Controller→Safety→UDP 경로를 그대로 사용했다.

| 단계 | 실제 관측 |
|---|---|
| 접근 | GPS/IMU 기반 RouteContext s=577.826m에서 정상 송신 재개; 30m 접근 범위 진입 후 RED/대상 head052 대응 |
| 적색 정지 | s≈596.537m, 정지선 약13.34m 전에서 실제 위치·속도 정지 유지 |
| 직진 녹색 | GREEN을 유효 판독했지만 좌회전이므로 STOP 유지. 중간 UNKNOWN/YELLOW도 허용하지 않음 |
| 좌회전 판독 | Camera source stamp 1792044862.231, RED+LEFT(mask9), valid, confidence0.91954 |
| 지도 상태 확정 | wall1792044862.646, source stamp1792044862.565, head C1256W000052/stop B2256W000252, RED+LEFT valid |
| 실제 재출발 | wall1792044863.835, EgoState v=0.741m/s; World 허용 확정 후 약1.19초 |
| 정지선 통과 | wall1792044879.227, RouteContext s=610.048m > 정지선609.877m |
| RViz | wall1792044862.909에 같은 head/stop의 RED\|LEFT MarkerArray 수신. `rviz-green-hold.png`는 실제 GUI의 GREEN 표시·대기 장면 |

이번 재시험의 localization reset_id는 처음부터 끝까지0이다. 실제 주석 영상
`red-annotated.jpg`, `green-hold-annotated.jpg`, `left-release-annotated.jpg`에서
상단 잘림 없이 각각 적색·직진 녹색·좌회전 화살표를 확인했다. RED+LEFT 때
차량이 출발했고 이후 카메라가 신호를 놓쳐도 유효 허용 상태에서 진입한
교차로 진행 확정으로 통과했다. 첫 유효 화살표 전에는 출발하지 않았다.

`validation-result.json`, `events.jsonl`, `frames/`, `source_hashes.json`이 근거다.
교차로를 지난 뒤 wall1792044907.118에 Q로 UDP handover하고 수동 제동·주차
입력으로 멈춘 후 ROS를 종료하고 시뮬레이션을 일시정지했다. 이후의 수동
구간은 자동 신호 대응 증거에서 제외한다. 로그인 정보는 증거 파일에 저장하지
않았다. 앞선 동시 종료의 원인은 확정하지 않았고, 독립 실행 재시험에서는
검증 종료까지 같은 현상이 재발하지 않았다.

**최종 판정:** 320°의 5번 구간에서 실제 YOLO/HSV 인식, 지도 대상 선택,
적색 자동 정지, 직진 녹색 대기, 좌회전 허용 자동 출발과 정지선 통과를
연속 확인했다. 1·2·3번의 기존 closed-loop 주행 증거는340°이므로 새 각도로
모든 교차로·날씨·속도까지 검증했다고 주장하지 않는다.


최종 실행 뒤 호스트 기본 Sensor 프로필도320°로 자동 저장되었다.
GPSList/IMUList 전체가 저장소 snapshot과 동일함을 확인했고 GPS/IMU scope
검증을 다시 수행하여 최종 hash를 중앙 증거에 반영했다. 재개 직후340°였던
hash와13:53의320° hash는 과거 관측으로 별도 보존했다.
