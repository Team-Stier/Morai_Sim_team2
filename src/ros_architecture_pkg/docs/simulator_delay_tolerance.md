# MORAI 지연 허용 완화 (2026-09-21)

사용자 요청: MORAI에서 정상 센서가 들어오는데 검출·차량 표시가 자주 사라지는
증상을 줄인다. MORAI 시뮬레이터 전용 개발 기본값이며 실차 적용값이나 측정으로
승인한 주행 한계가 아니다. 중앙 계약과 소비자가 읽는 설정을 함께 조정했다.

| 항목 | 이전 | 변경 |
|---|---:|---:|
| Localization IMU / 추정 결과 만료 | 0.30 s | 1.0 s |
| GPS freshness | 0.65 s | 2.0 s |
| Localization status / clock stall | 0.50 s | 2.0 s |
| LiDAR 통신 watchdog | 1.0 s | 3.0 s |
| 자세 보간 양 끝 간격 | 0.10 s | 0.30 s |
| 스캔 자세·장착 TF 대기 | 0.25 s | 1.0 s |
| 자세 이력 / 수신 만료 | 2.0 / 0.30 s | 5.0 / 1.0 s |
| 대기 스캔 수 | 3 | 16 |
| RViz 차량·점군 만료 / clock stall | 0.5 s | 2.0 s |
| LiDAR 단독 debug 표시 만료 / clock stall | 1.0 s | 2.0 s |

정상 입력은 준비되는 즉시 처리한다. 1초를 항상 기다리는 변경이 아니다.
LiDAR는 자세가 준비되어도 mount TF가 늦게 도착하면 동일 스캔을 큐에 유지하고
10 ms 정렬 타이머에서 재시도한다. 최초 스캔 수신부터 1초 안에 TF가 없으면
기존과 같이 invalid를 발행하며, 최신 TF로 대체하지 않는다.

원본 측정시각, reset_id 초기화, 잘못된 frame·NaN·quaternion·메시지 구조 검사는
유지한다. 실제 IMU 측정 간격이 0.30초를 넘는 단절은 재초기화 대상이다.
GPS 추측항법 한도 15초, 지면 제거의 지지 조건과 자차/ROI 설정은 변경하지 않는다.
`ready=false`, `stop_required=true`는 개발 상태를 나타내며 표시를 차단하지 않는다.
유효한 빈 검출은 장애물이 없다는 새 관측이므로 이전 장애물 표시를 지운다.

## 권위 및 적용

- Localization: `config/timestamp/timestamp_contract.yaml#development_localization_profile`
- LiDAR: `config/messages/lidar_runtime.yaml`
- 통신: `config/morai_interface/udp_ros_bridge.yaml` 및 그 투영인 bridge YAML
- RViz: `visualization_pkg/config/vehicle_display.yaml`, `lidar_debug.yaml`의 표시 전용 설정

설정은 해당 노드 시작 시 읽는다. 실행 중인 노드에는 재시작 후 적용된다.
허용 시간이 늘어난 만큼 단절 판정과 화면에서 마지막 관측이 사라지는 시점도
늦어진다. 마커의 원본 stamp를 갱신하거나 오래된 관측을 새 관측으로 발행하지 않는다.

## 검증 범위

합성 테스트는 0.35초 지연 자세와 늦은 mount TF의 복구, 0.60초 전달 지연 IMU의
원본 시각 보존, RViz 0.8초 지연 허용 및 2초 이후 만료를 검사한다.
중앙 timeout과 producer/consumer 설정의 일치, 기존 손상 데이터·reset·단절 경로도
함께 검사한다. 실제 MORAI에서 모든 사라짐 원인을 재현·해결했다는 의미는 아니다.

### 실행 결과

- 전체 catkin 빌드 성공.
- LiDAR 52, Localization 31, Visualization 33, MORAI Interface 74개 검사 통과.
- 중앙 계약 47개 검사, 인터페이스 다이어그램 일치, YAML/launch XML 검사 통과.
- 실행 중인 센서/Localization/LiDAR/RViz를 재시작하고 설정 적용을 확인했다.
- 첫 MORAI 15초 관측: Localization 599/599 valid, LiDAR 121/123 valid.
  동일 구간에 GPS innovation reset 기록이 있으며 단순 시간 만료로 단정하지 않는다.
  리스폰·재배치 시 이전 epoch의 점군을 지우는 동작은 유지한다.

- 이어진 20초 관측: Localization 759/759 valid, LiDAR 182/182 valid.
  reset_id=1 유지, 새 invalid/dropped 증가 없음. 이 구간의 연속 동작을 확인했으며
  모든 시뮬레이터 상황에서 사라짐이 없어졌다는 증거는 아니다.

로그와 수집 결과: `/home/paik/morai-artifacts/relaxed-guards-20260921/`.
