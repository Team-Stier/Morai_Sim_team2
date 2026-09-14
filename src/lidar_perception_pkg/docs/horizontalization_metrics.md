# 수평화 정량 평가

## 무엇을 측정하는가

검출기가 실제 사용한 회전행렬과 동일 stamp의 원본 점군을 대응하여 평가한다.
원본 점군에서만 RANSAC/TLS로 지면 후보를 한 번 선택한 뒤 **동일한 점 인덱스**에
기록된 회전을 적용한다. 보정 후에 유리한 점을 다시 고르지 않는다.
이 도구는 패키지 내부 개발 진단이며 주행 평가·판정의 소유자인
`runtime_evaluation_pkg`를 대체하지 않는다. 새로운 공개 ROS 출력은 없다.

| 지표 | 정의·단위 | 해석 |
|---|---|---|
| 수평 기준 잔류 기울기 | `atan2(sqrt(nx²+ny²), abs(nz))`, ° | 평면 법선과 수직축 사이 각도. 평지 기준이 있을 때만 수평화 오차로 해석 |
| 전방/좌우 경사 | `atan2(-nx,nz)`, `atan2(-ny,nz)`, ° | 동일 지면 후보의 X·Y 방향 경사 |
| 수평 높이 편차 RMS | `sqrt(mean((z-mean(z))²))`, m | 상수 높이 평면 대비 편차. 경사·점군 공간 범위의 영향을 받음 |
| 최적 평면 잔차 RMS | TLS 평면에 수직인 거리 RMS, m | 강체 회전 전후 동일해야 함. 수평화로 감소할 노이즈 지표가 아님 |
| 거리 보존 RMS | 회전 전후 점의 원점 거리 차이 RMS, m | 기록된 행렬의 형상 보존 확인 |
| 역회전 복원 RMS | `sqrt(mean(norm(RᵀRp-p)²))`, m | 기록된 회전 계산의 수치 안정성 |
| 유효 관측 비율 | 수집된 raw stamp 중 valid 관측과 대응된 비율 | 측정 구간의 파이프라인 처리율. 객체 인식 정확도나 UDP 손실률 아님 |
| 지면 지표 커버리지 | 모든 수집 raw 중 지면 평가가 가능한 스캔 비율 | 낮은 커버리지에서 소수 성공 표본만으로 성능을 주장하지 않음 |
| 지연 p50/p95 | 원본 scan stamp → 진단 도구의 관측 수신, ms | ingress 기반 전체 전달 지연. 별도로 C++ 검출 콜백 계산 시간 기록 |

각 지표에 스캔별 값과 전체 median/p95/min/max를 기록한다. 전후 기울기의
차이도 남기되 **정확도 % 또는 개선 성공률은 만들지 않는다**. 실제 평지의 독립
기준이 없으면 도로 경사와 센서 외부 파라미터 오차를 분리할 수 없기 때문이다.
형상 보존 지표는 기록된 행렬에 대한 double 정밀도 수학 검증이며 PCL의 모든
float32 포인트 출력 오차나 센서의 물리적 정확도를 측정한 값은 아니다.

## 지면 후보와 표본 제외

`config/horizontalization_metrics.yaml`은 **평가 전용**이다. 검출 ROI와 별도로
raw lidar_link X `[2,20]`, Y `[-5,5]`, Z `[-3,-0.5]` m에서 지면 후보를 찾는다.
최대 4,000점의 결정적 표본, RANSAC 150회, 거리 5 cm, 최소 100점,
최소 inlier 비율 45%, X/Y span 5/3 m, raw 평면 기울기 최대 30°를 요구한다.
단일 평면으로 설명하기 어려운 도로·연석·가림 상황은 평가에서 제외될 수 있다.
이 선택은 보정행렬을 사용하지 않는다. 실패 사유와 제외 개수도 결과에 포함한다.

표본이 없으면 `null`/N/A를 출력한다. 실패나 누락을 0° 오차로 넣지 않는다.
최적 평면 잔차는 같고 수평 기울기·높이 편차만 줄어드는 것이 평지에서 기대되는
결과다. 실제 경사로에서는 보정 후에도 경사가 남는 것이 정상일 수 있다.

## 실행과 산출물

보정 DBSCAN, LiDAR bridge/watchdog, GPS/IMU 및 Localization이 실행 중인
live ROS 환경에서 별도 터미널로 실행한다. 실행 중인 노드를 변경하지 않는다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun lidar_perception_pkg measure_horizontalization.py \
  --duration 20 --output /tmp/lidar-level-metrics
```

출력 디렉터리는 새 디렉터리 또는 빈 디렉터리여야 한다. 수집은 wall time 기준
1~120초이며 시작 연결 대기 0.5초, 마지막 관측 대기 1초를 추가한다.
원본 메모리 저장은 128 MiB로 제한하고 초과한 표본은 별도 제외 개수에 남긴다.
수집 중에는 큰 평면 계산을 하지 않고 수집 종료 후 분석한다.

- `report.html`: 기울기·높이 편차 전후 표와 시간 그래프.
- `summary.json`: 전체 통계, 처리율, 제외 사유, 실제 평가 설정.
- `frames.csv`: 스캔별 원본 ns 시각·평면/형상 지표·지연.
- `example_patch.npz`: 첫 유효 지면 후보의 동일 점들, 실제 회전행렬과 시각.
- `metrics.png`, `ground_patch.png`: 저장·공유용 그래프.

도구는 `/molit/internal/lidar_perception/horizontalization_metrics` 이름으로
raw scan, observation과 같은 패키지 private audit만 읽는다. ROS clock replay는
지원하지 않으며 `use_sim_time=true`이면 거부한다. 제어·TF·공개 평가 토픽을
발행하지 않는다.

## Private audit

`/lidar_perception_node/horizontalization_audit`는 `std_msgs/String` JSON이다.
유효한 검출에만 원 scan의 `stamp_ns`(정수 정밀도를 보존하는 문자열),
`leveling_enabled`, 실제 적용한 row-major `rotation[9]`, `processing_ms`를
발행한다. 구독자가 없으면 직렬화하지 않는다. 다른 패키지는 구독하지 않는다.
이는 센서 frame이나 공개 관측 타입을 추가하는 경로가 아니다.

## 검증

합성 평지에서 알려진 pitch가 제거되는지, 실제 도로 경사는 남는지,
평면 잔차·거리 보존, raw 점 선택의 보정행렬 독립성, 퇴화·빈 점군과 잘못된
회전행렬의 거부를 테스트한다. ROS 통합 테스트는 audit의 stamp와 행렬이
실제 수평화에 사용한 값과 일치하는지 확인한다.

## MORAI 측정 기록 (2026-09-14)

20초 동안 raw 154개, valid 관측 및 실제 회전행렬 대응 154개,
지면 지표 산출 154개(커버리지 100%), 제외·관측 누락 0개였다.
평면 inlier 비율 중앙값은 96.98%다. 독립적인 평지 정답은 사용하지 않았다.

| 스캔별 지표의 중앙값 | 보정 전 | 보정 후 |
|---|---:|---:|
| 수평 기준 기울기 | 0.49693° | 0.07238° |
| 전방 경사 | −0.47932° | 0.05461° |
| 좌우 경사 | 0.13111° | 0.04748° |
| 수평 높이 편차 RMS | 2.7189 cm | 1.2133 cm |
| 최적 평면 잔차 RMS | 1.1554 cm | 1.1554 cm |

보정 후 기울기 p95는 0.08421°. 전체 유효 관측 수신 지연은
median 131.19 ms / p95 141.18 ms, 검출 콜백 계산 시간은
median 22.75 ms / p95 32.97 ms였다. 소각도·현재 장면 측정이며 큰 경사,
급회전, 여러 도로 및 센서 장애 조건의 성능을 대표하지 않는다.

로컬 보고서:
`/home/paik/morai-artifacts/lidar-horizontalization-metrics-20260914/report.html`.
같은 디렉터리의 CSV/JSON/NPZ와 그래프로 전후 표본과 설정을 확인할 수 있다.
빌드 및 LiDAR 테스트(새 지표 테스트 5개 포함), audit ROS 연동 테스트,
중앙 계약 다이어그램 검사가 통과했다.
