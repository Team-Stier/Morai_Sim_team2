# 터널 주행 모델 불확실성 개선 — 2026-09-22

## 구현과 선택 근거

기존 9-state GPS/IMU 필터에 **가감속·회전 중 연속시간 모델 잡음**을 추가했다.
정지 드리프트용 bias 보정은 유지한다. 추가 센서·topic·TF·GT 관측은 없다.

녹화 주행에서 터널 진입 시점부터 속도 오차가 존재했다. 기존 `G Q Gᵀ`는
독립적인 샘플별 가속도 잡음이므로 dt가 작아지면 단위 시간당 기여도 작아진다.
이 항은 그대로 두고, 시각 불일치·미모델링 운동을 위한 별도 모델 항을 넣었다.
따라서 IMU 잡음 수치를 임의로 크게 바꾸는 것과 다르다. 이것이 모든 오차의
원인이라는 뜻은 아니며, ingress fallback 시각과 물리 기준점도 여전히 검증 대상이다.

보정된 world 가속도 크기를 a라 하면:

```
w = clip((a - 0.2) / (1.0 - 0.2), 0, 1)
q = 1.0 * w                 # m²/s³, 측정 잡음이 아닌 모델 spectral density
Q_pp = q * dt³/3 * I
Q_pv = Q_vp = q * dt²/2 * I
Q_vv = q * dt * I
```

위 세 임계값은 `config/localization.yaml`에 둔다. 일정 q일 때 이 모델 항은
적분 구간 분할에 불변이다. 전체 필터의 샘플 잡음/가변 자세까지 완전한
주기 불변이라고 주장하지 않는다. quiet/등속 구간에는 원래 bias 학습을 유지한다.
GPS가 있는 운동 구간에서 position–velocity 보정이 예측에 덜 묶이도록 하고,
blackout에서는 불확실성을 증가시킨다. 관측 없이 평균 위치·속도를 고정하지 않는다.
local odometry의 보수적 불확실성 누적에도 새 항을 포함한다.

시험 후 제외한 방법:

- 무조건적인 횡속도 0 제약: 같은 기록 최대 오차가 2.93 → 3.13 m로 악화되어 제거.
  실제 횡운동/기준점 차이가 있을 수 있어 no-slip을 정답으로 강제하지 않는다.
- 일정한 큰 모델 잡음: 기존 정지·등속 bias 회귀를 악화시켜 제외.
- LiDAR odometry: 현재 기록에 point cloud가 없고 UDP runtime gate가 닫혀 있어
  검증하지 않은 센서를 실행 경로에 추가하지 않았다. [FAST-LIO](https://github.com/hku-mars/FAST_LIO),
  [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)은 후속 후보이나 센서 시각·extrinsic·
  deskew와 [터널 기하 퇴화](https://www.cs.cmu.edu/~kaess/pub/Zhang16icra.pdf) 검증이 필요하다.

## 녹화 주행 A/B 결과 (개발 세트)

동일 GPS/IMU, 동일 초기 상태, 동일 원본 stamp로 baseline `78345f9`와 비교했다.
MORAI 실제 Ego 위치는 사용자가 요청한 **오프라인 평가에만** 사용한다.
GT와 trajectory를 정렬/fitting하거나, GT를 필터·런타임에 입력하지 않는다.
원본 GT 중 같은 timestamp의 중복은 제거하고, 보간 간격 150 ms 초과는 제외한다.
GPS age > 0.65 s인 동일 188개 추정 샘플을 비교했다.

| 수평 오차 | 기존 bias 개선판 | 이번 개선판 |
|---|---:|---:|
| 터널 RMSE | 2.203 m | 0.661 m |
| 터널 P95 | 2.833 m | 0.853 m |
| 터널 최대 | 2.929 m | 0.921 m |
| 터널 횡방향 RMSE (GT heading 기준) | 1.584 m | 0.124 m |
| 터널 횡방향 최대 절대값 | 2.674 m | 0.212 m |
| 전체 GT 겹침 구간 RMSE | 0.835 m | 0.276 m |

실시간 status 기준 이전 보고의 191개/2.189 m와 달리, 이 비교는 **추정 측정시각의
GPS age**로 두 알고리즘에 같은 구간을 적용한다. 상태 발행 시각과의 차이를 숨기지 않는다.

설정 선택에 이 기록을 사용했다. 독립 검증 세트가 아니며 일반화/대회 성능이 아니다.
q=0.5/1.0/2.0 민감도 시험의 터널 RMSE는 각각 1.201/0.661/0.547 m,
최대는 1.615/0.921/1.003 m였다. 최대 오차와 과도한 모델 잡음을 고려해 1.0을 선택했다.
독립 합성 가속도 scale 오차 시나리오에서는 3초 blackout 끝 오차가 2.394 → 0.578 m였다.

## 검증과 한계

- catkin 전체 build 성공. Localization/공유 메시지/중앙 계약 catkin 집계
  128 tests, errors 0, failures 0, skipped 1 (개인 MORAI profile 검사).
  YAML 31개, XML 46개 파싱과 중앙 interface diagram 검사 통과.
- 기존 stationary/등속/감속/재출발/경사/bias/reset 회귀 9개 유지.
- 새 모델 covariance PSD, 10/20/50/100 Hz 적분, 횡속도 보존, local 연속성,
  quiet 구간, YAML 일치, 비정상 설정, 독립 운동 모델 오차 회귀.
- 실제 ROS 합성 입력 → EgoState/Odometry/status/TF → visualization 소비자 검사.
- `stop_required=true`, 중앙 dead-reckoning 제한, clock/stale/reset 정책 유지.
- **아직 패스 플래닝에 안전하게 쓸 수 있다고 판정하지 않는다.** 다른 속도·회전·
  터널 길이·정지/재출발의 독립 주행과 횡방향 오차, 지연, covariance 일관성 검증이 필요하다.
  긴 무관측 구간의 절대 오차를 이 변경만으로 제한할 수 없다.
- 새 MORAI closed-loop 터널 재주행은 수행하지 않았다. 녹화 재현과 live 실행 확인을 구분한다.
- 이 PC에서 기존 센서 브리지/RViz를 유지하고 개선판 localization node로 교체했다.
  8초 관찰에서 IMU 345/GPS 196/EgoState 345개, TRACKING 및 `stop_required=true`.
  추가 4초 검사에서 160개 Ego/status 쌍이 원본 IMU stamp 및 두 동적 TF와 일치했고
  공유 메시지 validator를 통과했다. 입력은 GPS/IMU뿐이며 GT/LiDAR 입력은 없다.
- 별도 시각화 패키지 전체 ROS 테스트는 origin/main의 `WorldModel` 메시지 생성 등록
  누락으로 실패한다. 원래 사용자의 staged 수정은 보존하며 이 변경에 섞지 않았다.

## 재현

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4
source devel/setup.bash
catkin_make run_tests_localization_pkg run_tests_common_msgs_pkg run_tests_ros_architecture_pkg
catkin_test_results build/test_results/localization_pkg
python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py --check
python3 src/localization_pkg/test/evaluate_tunnel_bag.py \
  --bag /home/stier/.ros/localization-evaluation-20260922-mo6o5K/sensors_estimates.bag \
  --truth /home/stier/.ros/localization-evaluation-20260922-mo6o5K/ego_truth.jsonl \
  --core src/localization_pkg/src/localization_pkg/live_estimator.py \
  --config src/localization_pkg/config/localization.yaml --output /tmp/tunnel-replay.json
```

재현 도구는 ROS master에 접속하거나 publish하지 않는다. 런타임 queue/clock watchdog을
완전히 재현하는 rosbag-play 테스트는 아니며, source-time 정렬의 코어 회귀용이다.
GT 기록 자체는 저장소에 넣지 않는다. 입력 SHA와 요약은 `docs/evidence/`에 기록한다.

이 PC의 분리 작업 폴더는 `/home/stier/morai-tunnel-motion`이다. 원래 작업 폴더의
staged 수정을 보존했다. 이 폴더에서는 `catkin_make --build build-local`을 사용하고,
재실행 시 해당 폴더에서 `source devel/setup.bash` 후
`roslaunch localization_pkg localization_pkg.launch`를 실행한다.
동일 이름의 기존 localization node를 먼저 종료하고, 센서 브리지/기존 RViz는 유지한다.
