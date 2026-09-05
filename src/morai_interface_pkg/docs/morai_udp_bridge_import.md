# MORAI UDP → ROS 어댑터 이식 기록

## 원본

- 저장소: `https://github.com/Team-Stier/Morai_Sim-Stier-Team2`
- 기준 커밋: `f889f7f5ae47b51c4c5c0211c6a92a62398ca269`
- 원본 패키지: `morai_udp_bridge`
- 확인 결과: 위 커밋의 패키지 트리는 `feature/udp-bridge-state`의
  `85ef044141516856eb14fcd353cbd9ce294bf0e2`와 동일하다.
- 라이선스 주의: 원본 `package.xml`은 MIT로 표기하지만 저장소 루트 LICENSE와
  파일별 저작권 헤더는 확인되지 않았다. 외부 배포 전 팀이 출처와 라이선스
  고지를 다시 확정해야 한다.

Python 파서와 노드 구현은 추적 가능성을 위해 원본 네임스페이스
`morai_udp_bridge`를 유지하되, ROS 패키지 소유자는 아키텍처 계약에 따라
`morai_interface_pkg` 하나로 유지한다. 실행 진입점은 소유 표식을 검사해 다른
workspace의 동명 Python 패키지가 선택되면 fail-closed로 종료한다.

## 가져온 범위

- Camera MOR 청크 수신, JPEG 재조립, `CompressedImage` 발행
- GPS NMEA 수신, checksum 검증, `NavSatFix` 발행
- IMU 115-byte parser와 `Imu` 발행 어댑터
- 구형 EgoVehicleStatus parser와 twist 어댑터 소스
- VLP16 외부 Velodyne driver launch
- LiDAR points watchdog
- 위 기능의 ROS 무의존 parser/UDP loopback 테스트

이번 요청은 MORAI에서 ROS로 들어오는 어댑터만 대상으로 하므로 다음은
의도적으로 가져오지 않았다.

- `ctrl_cmd_node.py`, `udp_sender.py`, `ego_ctrl_cmd_packet.py`
- fake UDP sender와 fake control publisher
- ROS → MORAI 제어 launch/config/test

## 통합을 위해 바꾼 부분

- launch의 ROS 패키지 이름을 `morai_interface_pkg`로 변경했다.
- Camera/GPS/IMU/LiDAR frame ID를 중앙 TF 계약 이름으로 변경했다.
- receive fallback은 parse 이후 현재시각이 아니라 `recvfrom` 직후 ingress
  시각을 보존하도록 변경했다.
- Camera packet timestamp가 현재 MORAI에서 Unix epoch와 일치함을 확인해
  세 카메라의 기본 timestamp source를 `packet`으로 변경했다.
- 세 카메라의 live chunk index가 0부터 연속 증가함을 확인하고, 누락 chunk와
  JPEG SOI/EOI marker 오류를 fail-closed로 폐기한다.
- Camera source timestamp의 미래값·중복·역행을 폐기하고, 역행 시 내부 기준을
  초기화해 다음 정상 프레임에서 재동기화한다.
- GPS는 같은 UTC epoch의 RMC/GGA 중 고도와 fix quality가 있는 GGA만 발행해
  Localization이 동일 위치를 두 번 융합하지 않게 한다.
- LiDAR watchdog age 계산은 ROS clock과 분리된 monotonic clock을 사용한다.
- LiDAR launch를 이 호스트 Noetic Velodyne 1.7.0의
  `DriverNodelet`/`TransformNodelet` 실행 방식에 맞췄다.
- Velodyne 중간 scan은 `/molit/internal/morai_interface/lidar/packets`로 제한하고,
  nodelet 논리 producer와 rosgraph caller(manager)를 중앙 계약에 함께 기록한다.
- 누락됐던 `std_msgs` 의존성을 추가했다.
- IMU, LiDAR, legacy Vehicle Status는 검증 전 실행되지 않도록 launch 기본값을
  비활성화했다.

## 현재 실행 중 MORAI에서 확인한 범위

2026-09-03에 `25.S4.251001.MolitComp03_Linux` 프로세스를 설정 변경 없이
읽기 전용으로 확인했다.

| 채널 | 확인 결과 |
|---|---|
| Front Camera | `127.0.0.1:9290 → :9291`, ROS JPEG 발행, 약 7.37 Hz 관측 |
| Left Camera | `127.0.0.1:9292 → :9293`, ROS JPEG 발행, 약 6.97 Hz 관측 |
| Right Camera | `127.0.0.1:9294 → :9295`, ROS JPEG 발행, 약 7.51 Hz 관측 |
| GPS | `127.0.0.1:9090 → :7801`, RMC/GGA 쌍 중 GGA valid fix만 발행, 약 1.67 Hz 관측 |
| LiDAR | nodelet 3개와 내부 scan→points graph 기동 확인, `:2368` packet 미관측 |
| IMU | 수신 port와 packet 미확인 |
| Vehicle Status | 대회 packet/port 미확인 |
| `/clock` | 임시 ROS master 검증 중에도 미관측 |

임시 ROS master에서 Camera/GPS의 ROS publish까지 확인했다. GPS 필터 재검증에서는
raw 40문장 중 같은 epoch의 RMC 20건을 제외하고 GGA 20건만 발행했다. 위 속도는
짧은 벽시계 표본이며 승인된 stale 기준이 아니다. 설정 목표 20/5 Hz보다 낮고
시점에 따라 달라져 simulator 부하와 wall/simulation time ratio를 포함한 장시간
측정이 필요하다. TF 축, 센서 단위, 공분산 및 본선 네트워크 호환성도 별도
검증 대상이다.

## 금지 상태

구형 `ego_status_packet.py`는 일반 EgoVehicleStatus 229-byte 레이아웃이며
대회 `Competition Vehicle Status`에 없는 position과 lateral velocity도
파싱한다. 현재 ROS 출력에는 position을 싣지 않지만, 공식 대회 packet 명세와
일치하기 전에는 이 노드 자체를 실행하지 않는다.

IMU의 scale과 covariance는 원본 placeholder이고, LiDAR의 2368 port도
Velodyne 예제값이다. 두 채널 모두 live 검증 전 system bringup에 포함하지 않는다.
