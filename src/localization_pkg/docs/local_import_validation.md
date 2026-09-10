> 이 문서의 이식·진단 기록은 과거 단계다. 현재 GPS/IMU 개발 추정 구현과
> 검증 범위는 [TF Localization 검증](tf_localization_validation.md)을 따른다.

# Localization 패키지 로컬 이식 및 진단 검증

- 날짜: 2026-09-10
- 대상: 로컬 `PAIK`
- 이식 원본: `feature/localization_pkg`, `b1dfb34c9c60120b391e98c6b44802649dda13ce`

EKF 코어, 후보 설정, 기존 ROS adapter와 설계를 가져왔다. 중앙 계약에 없는
이전 메시지·node/topic/frame을 사용하는 adapter는 소스로 보존하고 빌드·설치·launch에서
제외했다. Noetic gtest 링크를 위해 테스트 main을 추가했다.

현재 중앙 계약의 `localization_node`에는 GPS/IMU 입력을 점검하고
`LocalizationStatus`만 발행하는 진단 실행 파일을 추가했다. 승인된 전체 I/O 표와
다이어그램은 유지하며, pose 추정 및 TF 발행은 잠긴 상태다. 공통 메시지 schema는
변경하지 않았다. 중앙 계약에는 진단 구현과 미구현 추정기의 차이를 기록했다.

2026-09-10 사용자 확정 위치인 IMU `(0, 0, 0)` m와 GPS `(0, 0, 1.3)` m는
[중앙 extrinsic 계약](../../ros_architecture_pkg/config/tf/sensor_extrinsics.yaml)에
반영했다. 과거 저장 프로필에서 가져온 값은 출처 이력으로 구분했다.
장착 위치 확인만으로 축·차량 원점 정합이나 추정기 활성화를 승인하지 않는다.

## 검증 범위

- Noetic catkin 전체 빌드와 EKF C++ 단위 테스트.
- 공개 producer-consumer 계약 및 구형 adapter 실행 방지 검사.
- 별도 ROS master의 합성 GPS/IMU와 `/clock`으로 진단 상태 메시지, 입력 거부,
  clock 정지·역행·0 reset, pose/TF 미발행을 검사하는 rostest.
- 중앙 TF/timestamp·공개 인터페이스·공통 메시지 계약 검사와 다이어그램 정합성 검사.
- 패키지 YAML/XML 파싱 및 diff 공백 검사.

검증 결과: 전체 catkin 빌드 성공. EKF C++ 4개, 인터페이스 2개, ROS 진단 4개가
모두 통과했다. Catkin은 중첩 gtest suite와 rostest 실행 항목을 포함해 Localization을
15 tests, 오류·실패·skip 0으로 집계한다. 중앙 아키텍처는 44개 중 43개 통과,
과거 `/home/stier` 프로필 부재로 1개 skip이며 현재 `/home/paik` 프로필 대조는 통과했다.
공통 메시지 24개와 다이어그램·YAML/XML·diff 검사는 모두 통과했다.

실제 MORAI UDP 수신을 관찰한 것과 합성 입력 rostest는 구분한다. 이 진단 검증은
map/odom 분리, 측정시각 기반 EKF, GPS blackout 실주행 또는 MORAI closed-loop
주행 검증을 뜻하지 않는다. 측정값의 진단 수용 횟수도 위치 추정 품질을 의미하지 않는다.
