# 센서 기반 위치 재설정

MORAI의 리스폰 이벤트나 숨은 상태를 읽지 않는다. 승인된 GPS/IMU 관측으로
**정지에 가까운 위치 재배치**를 추정하는 개발 기능이다. 설정의 원본은
`config/localization.yaml`의 `relocation`이며 일반 GPS gate(chi²=25)는 유지한다.

## 판정 순서와 기본값

1. 연속 GPS 간격이 0.65초 이하이고 IMU가 중앙 시간 계약의 최대 적분 간격
   이내로 들어와야 한다. 잘못된 frame, stamp, NaN, no-fix, 입력 단절은 증거를 지운다.
2. 현재 GPS와 필터 예측의 수평 차이가 15 m 이상이어야 한다. 직전 GPS부터의
   수평 이동에서 IMU 예측 이동을 뺀 차이도 15 m 이상이고, 그 사이 IMU 예측
   이동량은 3 m 이하여야 후보가 된다. 단순히 누적 오차가 큰 것만으로 시작하지 않는다.
3. 중력을 제거한 가속도 크기 2 m/s² 이하, 각속도 크기 0.5 rad/s 이하를
   유지해야 한다. GPS 보정용 공분산의 최대 주축 표준편차는 2 m 이하다.
4. 첫 점프를 포함해 GPS가 5개 이상, 첫 후보부터 최소 0.6초 동안 첫 새 위치의
   반경 1.5 m 안에 모여야 한다. 2초 안에 확인하지 못하면 후보를 취소한다.
   GPS 주기 약 0.2초에서는 다섯 번째 샘플까지 약 0.8초가 걸린다.

판정은 GPS 안테나 오프셋을 제거한 base_link 위치와 원본 측정시각을 사용한다.
IMU 가속도·각속도 조건은 GPS 사이에 들어오는 모든 유효 IMU에도 적용한다.

## 재초기화와 공개 계약

- 후보 중: 내부 관성 적분은 계속하지만 pose/Odometry/TF 발행은 보류한다.
  상태는 `RELOCALIZING`, 두 pose validity=false, `gps_fix_valid=false`,
  `stop_required=true`다. 모드와 validity를 검사하는 RViz는 잠시 차량을 숨긴다.
- 확인 후: GPS 위치와 동시각 IMU 자세로 map 위치를 재설정하고 이전 속도와
  map 필터 공분산을 초기화한다. 속도 0은 정지 후보의 초기 가정이며 공분산은 0이 아니다.
  `reset_id`를 한 번 증가시키고 다음 유효 IMU에서 새 pose/status/TF를 발행한다.
- 연속 odom 위치와 누적 불확실성은 보존하고 `map -> odom`을 새 위치에 맞춘다.
  새 GPS를 이전 odom 위치에 더해 차량을 순간 이동시키지 않는다.
- `EgoState`/`LocalizationStatus` 소비자는 기존 계약대로 epoch 변경 시 시간
  캐시를 비우고 새 epoch의 정확한 stamp 쌍을 기다린다. World Model·Route·Planner·
  Control·Safety도 invalid 상태와 reset을 따라야 한다. 현재 구현된 RViz 소비자를
  통합 테스트하며 미구현 소비자의 실제 주행 동작을 검증한 것으로 간주하지 않는다.
- 공개 node/topic/message schema는 추가하지 않는다. 상태 의미는 중앙
  `config/messages/core_messages.yaml`, TF와 측정시각은 중앙 TF/Timestamp 계약을 따른다.

## 의도적인 제한

GPS blackout 뒤 새 위치가 들어오는 것만으로 재설정하지 않는다. 새 위치로 바뀌는
순간의 연속 관측이 필요하므로 기능 시작 전에 발생한 리스폰도 소급 감지하지 않는다.
빠르게 움직이거나 회전하는 동안의 리스폰은 이 보수적인 정지 조건으로 놓칠 수 있다.
센서가 끊기거나 후보가 흩어지면 후보를 취소하고 기존 GPS gate/오류 경로로 돌아간다.

GPS 오작동이 새 위치에 일관되게 고정되는 현상과 실제 리스폰은 GPS/IMU만으로
완전히 구분할 수 없다. IMU가 조용하다는 사실만으로 정지를 증명할 수도 없다.
따라서 로그에는 `sensor relocation confirmed`로 기록하며 대회 주행 허가나
Ground Truth 확인으로 해석하지 않는다. 개발 상태의 `stop_required=true`를 유지한다.

## 검증 범위

순수 추정기 테스트는 재설정, 이전 속도 제거, odom 연속성, 단일 이상값,
흩어진 GPS, blackout, 중복/NaN/불확실한 입력, 운동과 최소 확인 시간을 검사한다.
ROS 합성 입력 테스트는 `RELOCALIZING`, reset_id 증가, 새 map pose, odom 연속성과
실제 RViz 소비자 검증 함수를 검사한다. 실제 MORAI 리스폰 및 closed-loop 주행은
별도 실측 검증이 필요하다.

### 2026-09-11 적용 결과

- catkin build 성공. 순수 추정기 10개, ROS 추정기 6개(재배치와 기존 RViz
  캐시의 epoch 전환 포함), 공유 상태 검증 14개, serialization 4개 통과.
  기존 C++ EKF 4개, 진단 노드 4개, Localization I/O 계약 2개도 통과했다.
- 중앙 공개 인터페이스 검사 29개, YAML/launch/manifest 파싱과
  `generate_interface_diagrams.py --check` 통과.
- 전체 계약 검사는 모두 통과한 상태가 아니다. 기존 공통 계약 검사에 남은
  `schema_implemented_runtime_not_implemented` 및 모든 TF 비활성 기대 2건은
  현재 개발 추정기/TF 활성화 상태와 불일치한다. 기존 작업 폴더에서도 실패를 재현했다.
  TF/Timestamp 검사 17개 중 1개는 저장 센서 프로필 근거 SHA 불일치로 실패한다.
  이번 재배치 기능을 위해 기존 근거 해시나 TF 활성화 조건을 변경하지 않았다.
- 실행 폴더에서 빌드 후 Localization 노드를 재시작했다. 12초 실측에서
  GPS 약 5.01 Hz, IMU 약 39.53 Hz, pose와 각 동적 TF 약 39.38 Hz를 확인했다.
  상태 474개 모두 TRACKING, pose/status 473쌍 검증 오류 0건이었다.
  [실측 기록](evidence/sensor_relocation_runtime_20260911.json)을 함께 보존한다.
- 실제 MORAI 리스폰은 수행하지 않았다. 합성 입력에 대한 자동 재설정과 실제
  센서 수신·상태·출력 확인을 구분하며 `stop_required=true`는 계속 유지한다.
