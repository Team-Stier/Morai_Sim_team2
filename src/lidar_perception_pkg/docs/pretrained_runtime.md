# 2026-09-14 로컬 MORAI 실행 기록

이 기록은 사전학습 모델 연결과 실행 성능의 개발 검증이다. 정답 라벨이 없어
검출 정확도·recall·성별/연령 분류 또는 주행 성능을 검증한 기록이 아니다.

## 환경과 실행

- Ubuntu 20.04, ROS1 Noetic, Python 3.8, RTX 3060 12GB, NVIDIA 535.230.02.
- PyTorch 2.4.1+cu118, spconv-cu118 2.3.6, OpenPCDet v0.5.2 + 저장된 호환 patch.
- 공식 nuScenes PointPillars-MultiHead 가중치 SHA256 확인 및 strict state load 성공.
- MORAI와 같은 GPU를 사용한다. 원래 `paik` worktree는 보존하고
  `/tmp/morai-lidar-learned`에서 별도 빌드·실행했다.
- 기존 raw LiDAR bridge/watchdog을 유지하고 검출기·LiDAR 표시 노드만 전환했다.
- 실행: `lidar_perception_pkg/learned_lidar.launch`,
  `visualization_pkg/lidar_debug.launch`. 점수 기준 0.3, 기존 ROI 유지.

## 실제 입력과 결과

처음 저장한 raw XYZI 스캔은 13,788점, frame `lidar_link`, intensity 0–156이었다.
가중치 로드 약 8.3초, 첫 추론 약 858ms, 같은 점군의 후속 추론 약 68–85ms였다.
이 스캔에서는 ROI 내 0.3 이상의 지지점 있는 박스가 없었다.

최종 실시간 수집은 raw 구독 준비 0.5초 후 관측을 15초 수집했다.

| 항목 | 결과 |
|---|---:|
| raw 수신 (구독 준비 구간 포함) | 138 scans |
| 처리 결과 / objects_valid=true | 134 / 134 |
| 원 raw stamp와 일치 | 134 / 134 |
| 공유 메시지 검증 오류 | 0 |
| 차량 car 관측 누적 | 21 |
| traffic_cone / barrier 관측 누적 | 5 / 2 |
| 보행자 관측 | 0 |
| RViz MarkerArray / text label 누적 | 135 / 28 |
| 처리 지연 p50 / p95 / p99 (2Hz status 표본) | 67.9 / 72.0 / 72.3 ms |

위 클래스 수는 여러 스캔에서 반복된 관측 수이며 실제 고유 객체 수가 아니다.
RViz에서도 파란 VEHICLE 박스와 원 모델 클래스/점수를 확인했다. 표시가
흔들리고 실제 차량을 construction_vehicle 등으로 분류하는 후보도 나와,
현재 상태는 안정적인 차종/보행자 인식으로 볼 수 없다. 전체 객체 검출도
보장하지 않는다. 임계값을 낮춰 정확도가 개선됐다고 주장하지 않는다.

상태는 DEGRADED, ready=false, stop_required=true였다. 모델 로드 중 최신
입력 하나만 남기는 drop이 발생했으며 종료 표본의 dropped_count=60은 시작
시점부터의 누적값이다. 구간 내 60개를 놓쳤다는 뜻이 아니다.

개발용 RViz Fixed Frame은 lidar_link이며 변환 TF를 새로 발행하지 않았다.
source stamp·frame·output·실제 XYZI 입력과 표시 경로를 검증했다.
센서 장착 물리 정합, 보행자 recall, multi-sweep 성능과 MORAI closed-loop
주행은 검증하지 않았다. 기존 calibration/freshness 미검증 gate를 유지한다.

## 파일과 자동 검증

로컬 실행 원본: `/home/paik/morai-artifacts/lidar-pretrained-20260914/`
(`runtime.json`, `rviz.png`, `preflight_predictions.npz`, 로그/환경 목록).
이 경로는 로컬 artifact이며 Git에 센서 데이터나 가중치를 넣지 않는다.

catkin 빌드 성공. common_msgs_pkg/lidar_perception_pkg/visualization_pkg 대상
72개 테스트가 error/failure/skip 없이 통과했다. launch XML/YAML 파싱,
중앙 계약 생성 검사와 git diff whitespace 검사도 통과했다. 공개 topology가
같아 Mermaid/SVG/PNG 내용은 유지하고 중앙 계약 source hash만 갱신했다.

모델 로드 실패·추론 실패·예산 초과·추론 중 clock reset 등은 주입 backend를
사용한 테스트 결과다. 실제 GPU 오류나 라이브 UDP 단절을 발생시킨 시험과
혼동하지 않는다. 다음 정확도 개선에는 여러 시나리오에서 허용 센서 데이터를
수집·라벨링하고, 주행 회차를 분리한 평가와 추가 학습이 필요하다.
