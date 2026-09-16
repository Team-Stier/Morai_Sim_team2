# GPS innovation 기준의 즉시 위치 재설정

사용자가 선택한 시뮬레이터 정책은 **GPS innovation χ²가 25를 넘으면 그 GPS를
리스폰 위치로 간주하고 즉시 수용**하는 것이다. 설정은 `config/localization.yaml`의
`gps_innovation_gate_chi2` 한 곳에서 읽는다. 25는 미터 단위 거리가 아니라
예측·관측 공분산으로 정규화한 오차 제곱값이다.

## 실행 동작

1. GPS 안테나 오프셋을 제거한 위치와 같은 측정시각의 예측 위치로 innovation을 계산한다.
2. χ² ≤ 25이면 일반 Kalman 보정을 적용한다.
3. χ² > 25이면 그 GPS 위치로 map pose를 즉시 재설정한다. 이전 속도를 0으로
   초기화하고 map 필터 공분산을 초기값으로 설정한다. 예: 40.3이면 즉시 재설정한다.
4. `reset_id`를 한 번 증가시키고 다음 IMU에서 새 pose/status/TF를 내보낸다.
   odom 위치와 누적 불확실성은 유지하며 `map -> odom`을 새 map 위치에 맞춘다.

GPS 여러 개 확인, 정지·가속도·각속도 조건, 15 m 최소 점프, 최소 대기시간,
재설정 간격 제한은 없다. GPS blackout 이후 첫 GPS도 기준을 넘으면 즉시 수용한다.
이전 후보 판정 모듈과 관련 YAML 파라미터는 제거했다.

## 공개 계약과 표시

공개 node/topic/message schema는 동일하다. 동작 의미는 중앙
`config/messages/core_messages.yaml`, TF와 시각은 중앙 TF/Timestamp 계약을 따른다.
`EgoState`와 `LocalizationStatus`의 reset_id가 바뀌면 RViz 등 소비자는 이전
pose/status 캐시를 비우고 새 epoch의 일치하는 쌍으로 갱신한다. 현재 정책에는
여러 GPS를 기다리는 `RELOCALIZING` 단계가 없다.

로그에는 `GPS innovation reset (chi2=... > 25.000)`과 reset_id를 남긴다.
이 동작은 실제 MORAI 이벤트를 읽지 않는 시뮬레이터용 가정이다.
기존 개발 상태의 `stop_required=true`와 원본 측정시각을 유지한다.

## 검증

순수 추정기 테스트는 첫 GPS의 즉시 수용, 주행 중 재설정, 15 m 미만 오차,
25 경계값, blackout 복귀, 연속 초과 GPS와 일반 보정을 검사한다.
ROS 테스트는 GPS 한 개만으로 epoch가 바뀌고 odom이 연속이며 기존 RViz
소비자가 새 epoch의 pose/status를 표시하는지 확인한다.

기존 전체 계약 검사에는 런타임 구현 상태·TF 활성화 기대값 2건과 저장 센서
프로필 근거 SHA 1건의 불일치가 있다. 이 기능 변경에서 해당 근거나 gate를
수정하지 않는다.

### 2026-09-11 적용 결과

- catkin build, 순수 추정기 8개, ROS 추정기 6개, 기존 C++ EKF 4개,
  진단 노드 4개와 Localization I/O 계약 2개 테스트 통과.
- 중앙 공개 인터페이스 29개, 공유 상태 검증 14개, 다이어그램 해시 및
  YAML/launch/manifest 파싱 통과. TF/Timestamp 17개 중 기존 근거 SHA 1건은 실패한다.
- GPS 하나를 입력한 ROS 테스트에서 즉시 reset_id가 바뀌고, 새 위치의
  pose/status 및 기존 RViz 소비자의 epoch 전환을 확인했다.
- 실행 중인 Localization을 새 코드로 재시작하고 이전 relocation 파라미터를 지웠다.
  12초 실측에서 초기화 대기 상태 120개를 받았고 GPS/IMU는 0개였다.
  시뮬레이터는 00:47:46경 Vulkan ImagePool 충돌로 종료된 상태였다.
  따라서 실제 입력에 대한 재설정은 이 실측 구간에서 검증하지 못했다.
  [실행 기록](evidence/sensor_relocation_runtime_20260911.json)을 보존한다.
