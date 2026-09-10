# MORAI 입력 호환성 점검 및 수정 기록

## 범위와 문서 근거

작업 브랜치: feature/public-interface-contract-lidar-detection. main 수정·병합 없음.
루트 README §12의 공개 계약 변경 규칙은 producer, consumer, 메시지, launch,
config, 문서와 통합 테스트를 함께 갱신하도록 요구한다. AGENTS.md의 파일 배치는
검증 근거를 docs/에 두도록 정한다. 이 문서가 이번 수정 기록이다.

## 판단

[파일] 브리지는 VLP16 UDP → Velodyne driver → PointCloud2 경로를 제공한다.
검출 노드는 /molit/sensors/lidar/points, sensor_msgs/PointCloud2를 구독하고
lidar_link, little-endian FLOAT32 x/y/z 필드를 검사한다. 추가 intensity/ring
필드는 허용하며 검출에는 XYZ만 사용한다. UDP 수신 자체는 검출 노드가 하지 않는다.

[미확정] README와 코드의 인터페이스는 연결 가능하지만 실제 대회 packet,
원점·축·활성 센서 설정 검증은 남아 있다. 코드만으로 수신 성공을 보장하지 않는다.
단독 검출 launch는 UDP 브리지와 watchdog을 켜지 않는다. 둘 다 기본 비활성이다.
점군만 들어오고 status가 없거나 False이면 검출 출력은 invalid가 된다.

## 격리된 센서 연결 시험

기준: morai_interface_pkg/docs/sensor_connection_runbook.md. ROS Noetic에서
catkin 빌드와 devel/setup.bash source를 완료한 후 서로 다른 터미널에서 실행한다.
system bringup의 비활성 채널이나 TF 발행 설정을 변경하는 절차가 아니다.

```bash
# MORAI destination IP = ROS PC IPv4. port는 실제 설정에 맞춘다.
# 아래 2368, 600 RPM은 저장소 개발 기본값이며 실제 설정과 대조해야 한다.
roslaunch morai_interface_pkg lidar_bridge.launch enable:=true port:=2368 rpm:=600
roslaunch morai_interface_pkg lidar_watchdog.launch enable:=true
roslaunch lidar_perception_pkg lidar_perception_pkg.launch
```

확인 명령:

```bash
rostopic hz /molit/sensors/lidar/points
rostopic echo -n 1 /molit/sensors/lidar/points/header
rostopic echo -n 1 /molit/sensors/lidar/points/fields
rostopic echo -n 1 /molit/sensors/lidar/status
rostopic echo -n 1 /molit/perception/lidar/observations
rostopic hz /molit/perception/lidar/status
```

입력 목표는 약 10 Hz, frame은 lidar_link, status는 True다. 실제 XYZ 축은
전방·좌측·높이가 다른 물체로 확인한다. frame 문자열만으로 축 검증을 대신하지 않는다.
raw 수신이 정상이어도 ROI 밖의 객체는 검출되지 않는다. 현재 z 범위 -0.5~1.0 m는
기존 코드의 센서 기준 범위다. 저장 프로필의 높이 1.5 m가 실제 지면 기준 높이와
일치하고 평탄·수평 장착인 조건에서는 지면 부근 낮은 물체가 이 범위 밖일 수 있다.
그 조건은 미검증이므로 임의로 ROI를 변경하지 않고 실제 점군으로 조정한다.

## 수정 내용

1. 수신 watchdog(steady clock, 1초)과 scan age(canonical ROS clock)를 분리했다.
   max_scan_age_sec=0은 아직 측정 기준이 없음을 뜻한다. 과거 scan의 age는 상태에
   보고하고 freshness_verified=false, ready=false를 유지한다. 주행에 사용하지 않는다.
2. 상태는 0.5초 타이머에서만 발행한다. scan/transport callback은 상태값만 갱신한다.
   오류 전파도 다음 heartbeat까지 최대 한 주기 지연될 수 있다.
3. LiDAR 상태 토픽의 구현 상태와 여러 README의 메시지 구현 현황을 동기화했다.
4. 실제 입력과 같은 XYZ+intensity 점군, status 필요 조건, 2 Hz heartbeat 및
   미확정 age 정책의 회귀 테스트를 추가했다.

## 검증과 사용 한계

로컬 Python 계약 검사는 C++ 빌드 증거가 아니다. Noetic CI 결과와 live UDP,
실제 검출 결과는 각각 확인해야 한다. TF/보정/시각 검증 전에는 항상 ready=false,
stop_required=true이며 이 검출기를 추가했다고 자율주행 가능 상태가 되지 않는다.
지면·free-space·occupancy·속도 추정은 이번 범위에 포함하지 않는다.

수정 후 로컬 검사: 공유 메시지·정책 29개 통과, 아키텍처 40개 중 39개 통과와
환경 의존 1개 skip. 중앙 다이어그램·해시 검사 및 git diff --check 통과.
작성 시 첫 Noetic CI는 의존성 설치 진행 중이며 C++ 빌드·rostest 성공으로
보고하지 않는다. 후속 push에 대한 CI가 최신 코드의 실행 검증 기준이다.
