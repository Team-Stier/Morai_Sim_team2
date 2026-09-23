# GPS 음영 구간 정지 드리프트 개선 — 2026-09-22

## 증상과 확인 범위

- [사용자 보고] 터널에서 정지 중 위치가 흐른다. GPS 수신 구간은 정상이며,
  이전 영상은 GPS가 수신되는 상태였다. 터널 진입 후 정지인지 처음부터
  정지한 상태인지 확실하지 않아 두 경우를 모두 시험했다.
- [파일] 기본 launch는 `localization_estimator_node.py`와
  `localization_pkg/live_estimator.py`를 실행한다. IDE에 열려 있던
  `localization_ekf.cpp`는 기본 추정 경로에 연결되지 않은 과거 15-state 라이브러리다.
- [파일] 기존 실행 추정기는 위치·속도 6-state이며, 가속도 편향을 추정하지 않았다.
  자세로 회전한 가속도에서 중력을 뺀 값과 남은 속도를 그대로 적분했다.
- [추론] 일정한 잔여 가속도 b가 있으면 위치 오차에 약 `0.5*b*t²`가 더해진다.
  편향 외에도 gravity/축/자세 오차, 초기 잔여 속도, 수신시각 기반 지연이 후보다.
  현장 bag이 없으므로 실제 원인을 하나로 확정하지 않는다.

## 적용한 변경

상태를 `[p_map, v_map, b_accel_body]`의 9차원으로 확장했다. 자세는 기존과 같이
IMU quaternion 관측을 사용한다. `R*(a_measured-b)-g`로 이동을 예측하고,
GPS 잔차가 position–velocity–bias 교차 covariance를 통해 편향도 보정한다.
주행 중 GPS도 학습에 사용하므로 터널 진입 전 반드시 정지할 필요는 없다.

편향은 일정값으로 확신하지 않고 random walk로 모델링한다. 연속 편향 잡음의
position/velocity/bias 적분 및 교차항을 covariance에 반영한다. GPS가 없는 동안
편향 평균은 관측 없이 바꾸지 않으며, 위치·속도 불확실성은 증가한다.
동일 자세에서 학습한 편향이 자세 오차까지 일부 흡수할 수 있지만, 이것은
IMU 축·중력·자세 정합의 검증이나 완전한 관성 보정을 대신하지 않는다.

추가로 새 GPS마다 아래 조건을 모두 만족할 때 약한 속도 0 관측(ZUPT)을 적용한다.
임계값은 `config/localization.yaml`의 개발용 값이며 실측 조정이 필요하다.

- 연속 GPS 2초 이상, 최소 5개. 중앙 integration bound보다 긴 fix 간격이면 재시작.
- 레버암을 제거한 GPS 위치가 창 첫 위치에서 8 cm 안에 있음. 시작·끝만 비교하지 않음.
- GPS 모델 covariance의 최대 표준편차 0.6 m 이하, 추정 속도 0.15 m/s 이하.
- 같은 기간 IMU의 보정 후 가속도 크기 0.15 m/s² 이하와 각속도 0.02 rad/s 이하.
- 속도 관측 표준편차 0.10 m/s로 Joseph covariance update를 사용함.

정지 여부는 임계값을 사용하는 모델 판단이다. 센서 해상도 아래의 아주 느린
이동·GPS 값 고정 고장과 실제 정지를 완전히 구분할 수 있다고 주장하지 않는다.
GPS 위치 관측과 겹치는 창에서 얻은 정지 판단은 독립적인 차속 센서가 아니므로
ZUPT covariance 역시 경험적 모델이다. 실제 데이터로 일관성을 검증해야 한다.

GPS 단절 중에는 ZUPT를 반복하거나 위치를 고정하지 않는다. 이전 GPS를 현재
위치 정답으로 재사용하지 않고, IMU가 조용하다는 이유만으로 새 정지를 확정하지 않는다.
이는 등속 이동과 정지의 IMU가 같을 수 있기 때문이다. 출발 가속도도
GPS가 없는 상태에서 새 편향으로 학습하지 않는다.

GPS innovation 기반 재배치 시에는 편향과 정지 근거를 함께 초기화한다.
기존 odom 위치·불확실성 누적은 보존한다. clock/IMU 단절 reset에서는 기존 정책대로
전체 필터를 초기화한다. 공개 topic/type/frame/stamp/reset 의미와 dependency 방향,
당시 구현은 GPS 재배치 gate, 중앙 15초 dead-reckoning budget 및 `stop_required=true`를 유지했다.
후속 2026-09-23 사용자 요청으로 시간 제한은 제거했으며 `stop_required=true`는 유지한다.

## 다른 개선 수단과 우선순위

| 방안 | 기대 효과 | 적용 조건·한계 | 이번 범위 |
|---|---|---|---|
| 가속도 편향 추정 | 정지·주행 모두에서 이중 적분 오차 감소 | GPS 학습 시간과 편향 안정성이 필요 | 구현 |
| GPS+IMU 기반 ZUPT | GPS가 있을 때 잔여 속도와 편향 수렴 개선 | 음영 구간의 새 정지를 독립적으로 알 수 없음 | 구현 |
| Competition Vehicle Status의 전후 차속 | 음영 구간 정지·재출발을 직접 구분, 종방향 속도 보정 | 공식 packet·단위·축·freshness 검증과 중앙 활성화 필요. 현재 사용 금지 | 다음 우선 후보 |
| LiDAR 벽/노면 제약 | 횡방향·높이·방향각 drift 감소 | 평행한 두 벽만으로 종방향 이동을 관측하기 어려움 | 검토 |
| LiDAR 특징 기반 상대 odometry | 돌출물·기둥·표지 같은 특징으로 종방향도 보정 | scan deskew, 동적 객체 제외, 겹침률·잔차·Hessian 고유값 기반 퇴화 검사가 필요 | 검토 |
| 카메라 특징 이동 | 정지·이동 판별의 별도 근거, LiDAR 취약 방향 보완 | 조명·안개·동적 차량·원거리 특징, 관측 계약과 Localization 입력 경계 변경 필요 | 검토 |
| 차량 비홀로노믹 제약 | 정상 주행 시 횡/수직 속도 오차 감소 | 미끄러짐·경사·회전·충돌에서 잘못된 가정 가능. 종방향 정지 자체를 알 수 없음 | 기본 적용 보류 |

권장 후속 조합은 검증된 차속을 종방향 관측으로 쓰고, LiDAR가 실제로 관측하는
방향만 선택적으로 보정하는 것이다. 벽 정합 성공만으로 모든 축 covariance를
작게 만들거나 차량을 차로 중심에 강제로 붙이면 안 된다. 정적 지도·전역경로·
체크포인트를 차량 위치 Ground Truth로 사용하지 않는다.

현재 승인·활성화된 GPS/IMU 입력만 사용하는 변경이므로 중앙 공개 계약 변경은 없다.
LiDAR 상대 odometry나 새 영상 관측을 실제 연결하려면 중앙 계약에 알고리즘 의미,
producer/consumer 영향, frame/time/quality와 활성화 조건을 먼저 제안해야 한다.
Camera/LiDAR 관측 패키지에 별도 전역 world model을 만들지 않는다.

## 검증 결과

[합성 비교 원본](evidence/blackout_drift_synthetic_20260922.json)의 baseline은
`3e5c738da3a50daac713f15d448edd4090964538`이다. 같은 입력을 과거 코드와 현재 코드에
각각 넣었다. IMU 50 Hz, GPS 5 Hz, GPS 수신 20초 뒤 blackout 15초,
body 가속도 편향 `[0.04,-0.025,0.015] m/s²`를 사용했다. GPS와 자세는 이상적인
합성 관측이다. 정지·주행 판단을 돕는 Ground Truth는 필터에 전달하지 않았다.

| 시나리오 | 지표 | 이전 | 개선 후 |
|---|---|---:|---:|
| 정지 후 GPS 단절 | 15초 동안 map 위치 이동량 | 6.831 m | 0.0167 m |
| 4 m/s 주행, GPS 단절, 2초 감속 후 정지 | 이후 13초 동안 map 위치 이동량 | 6.607 m | 0.0849 m |
| 계속 4 m/s 등속 주행 | blackout 15초 후 위치 오차 | 6.955 m | 0.0839 m |

이 수치는 고정 편향에 대한 회귀 결과이며 터널 실측 정확도나 일반적인 개선율이 아니다.
개선 후 수평 위치 표준편차는 각각 약 5.45/6.27/6.27 m로 보수적으로 남는다.
작은 실제 합성 오차를 근거로 covariance나 안전 gate를 임의로 낮추지 않았다.

2026-09-22, Ubuntu 20.04 / ROS Noetic arm64 격리 컨테이너:

- Localization, ros_architecture, common_msgs, hd_map, visualization 5개 패키지 catkin build/install 성공.
- Localization·중앙 계약·공유 메시지 테스트: catkin 결과 119개 집계, errors 0,
  failures 0, skipped 1. 개인 MORAI 저장 프로필이 없는 항목만 skip.
- 새 core 회귀 9개: 편향+정지/주행/감속, 등속 오판 방지, 경사 자세, 단일 GPS,
  재출발, relocation/reset, GPS 불확실성/중간 이동, 잘못된 파라미터.
- 실제 ROS 합성 publisher → estimator → EgoState/Odometry/status/TF 및
  visualization consumer 검사 성공. NaN·역순·clock stall/reset·GPS 복귀도 기존 검사 통과.
- 중앙 diagram/공개 계약 검사 통과. 저장소 YAML 31개, launch/test/manifest XML 46개 파싱 성공.
- 실제 대회 UDP, 현장 bag replay, MORAI 터널 closed-loop는 이번에 수행하지 않음.

재현 명령(ROS 환경):

```bash
source /opt/ros/noetic/setup.bash
PYTHONNOUSERSITE=1 catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
PYTHONNOUSERSITE=1 catkin_make run_tests_localization_pkg run_tests_ros_architecture_pkg run_tests_common_msgs_pkg
catkin_test_results --all build/test_results
python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py --check
```

합성 비교만 실행하려면 numpy/pyproj가 있는 Python에서:

```bash
git show 3e5c738da3a50daac713f15d448edd4090964538:src/localization_pkg/src/localization_pkg/live_estimator.py > /tmp/localization_before_drift.py
PYTHONPATH=src/localization_pkg/src python3 src/localization_pkg/test/evaluate_blackout_drift.py \
  --baseline-file /tmp/localization_before_drift.py \
  --baseline-commit 3e5c738da3a50daac713f15d448edd4090964538 \
  --output /tmp/blackout_drift.json
```

현장에서는 GPS 수신 중 20–30초 구간을 포함하여 (1) 정지 후 blackout,
(2) 주행 중 blackout 후 감속·정지·재출발, (3) 등속 주행, (4) GPS 복귀를
기록한다. 원본 IMU/GPS, localization의 세 출력과 TF를 같은 bag에 보존한다.
정지 이동량뿐 아니라 재출발 지연, 등속 이동량, covariance, reset_id와 소비자
유효성도 비교해야 한다. GPS가 처음부터 전혀 없으면 INITIALIZING이 정상이며,
기존 15초 제한은 2026-09-23 사용자 요청으로 제거됐다. 현재는 GPS 미수신 시간만으로 LOST가 되지 않는다.

## 참고 근거

- [OpenVINS ZUPT 설명](https://docs.openvins.com/update-zerovelocity.html):
  관성 기반 정지 판단의 등속 오판과 별도 속도·영상 근거의 필요성.
- [OpenVINS IMU propagation](https://docs.openvins.com/propagation.html):
  가속도 편향과 random walk 모델.
- [Zhang et al., ICRA 2016](https://www.cs.cmu.edu/~kaess/pub/Zhang16icra.html):
  기하 제약의 퇴화 분석과 관측 가능한 방향의 처리.
- [Tunnel facility-based localization](https://arxiv.org/abs/2012.13168):
  터널 구조물 특징을 활용하는 LiDAR 접근.
- [규정] 저장소 README의 허용 UDP·GPS blackout·Ground Truth 금지 베이스라인을
  따랐다. 공식 규정 URL은 이번 웹 조회에서 열리지 않아 최신 변경을 재확인하지 못했다.
  `참고파일들/`의 센서 프로필·전역경로·sample scene 원본은 조회만 했고 수정하거나
  추정기의 위치 관측으로 사용하지 않았다.
