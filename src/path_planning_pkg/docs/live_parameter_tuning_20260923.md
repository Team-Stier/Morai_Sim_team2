# 현재 주행 기반 정지 감소 조정 (2026-09-23)

기준 코드는 `f325c70`. 실행 중인 주행에서 약 90초의 승인된 ROS 출력만
별도 bag으로 기록했다. 실행 노드를 재시작하거나 ROS parameter server를
변경하지 않았으므로 아래 설정은 다음 실행부터 적용된다.

## 변경

- `minimum_gain_sec`: 1.5 → 1.0. ETA·차로변경·승차감 비용을 합산한
  이득이 1초 이상이면 검증된 대안을 선택할 수 있다. committed 경로 유지와
  0.5초 경로 유지 조건은 그대로다.
- `world_model_input_age_sec`: 개발 RDDF 형상 전용 모드에서 0.7초.
  기존의 공통 0.5초 gate에서 WorldModel만 분리했다. 위치·경로 입력과
  비개발 모드는 0.5초를 유지한다. 새 파라미터가 없는 설정도 0.5초를 사용한다.
  무효 scene, 미래 stamp, localization epoch 불일치, 0.7초 초과는 정지한다.

생산자 WorldModel과 공유 메시지·TF·원본 stamp는 변경하지 않는다.
소비자 중 Planner의 개발 프로필만 바뀐다. 일반 공개 scene timeout 0.5초,
시각화·Safety의 기존 동작은 그대로이며, 개발 예외는 중앙
`config/messages/frenet_runtime.yaml`에 명시한다. 객체의 과거 점군을 현재
관측으로 덮어쓰지 않고 기존 충돌 검사에서 실제 source age로 위치를 예측한다.
추가로 허용하는 최대 0.2초의 관측 지연은 예측 불확실성을 늘릴 수 있다.

## 기록과 비교

- 새 bag 89.77초의 ControllerStatus 899개 중 tracking 861,
  planned_stop 26, upstream_stop_required 12개. 정지 요구 상태 약 3.8초이며
  실제 정차 시간과 다르다. tracking 횡오차 절댓값 p95 0.292m, 최대 0.509m.
- 49.4~50.4초의 planning 입력 정지 6개는 scene age 0.514~0.651초였다.
  같은 위치·scene·경로를 Node.plan/Node.publish에 넣어 다시 계산했을 때
  기존 0.5초는 6개 모두 정지, 개발 0.7초는 6개 모두 `feasible` 주행 경로였다.
  제동·충돌 검사를 생략하지 않았다. 원래 제어기의 정지 요구 약 1.2초 구간에
  해당하지만 차량 응답을 바꾼 closed-loop 절감량은 검증하지 않았다.
- 이전 7분 44초 bag의 keep/대체 후보 비용이 모두 유한한 audit 73개 중
  최소 이득 1.5초를 넘는 것은 9개, 1.0초를 넘는 것은 28개였다.
  이는 후보 자격 비교이며 실제 차로변경 횟수나 시간 절감량은 아니다.
  새 90초 기록에는 해당 비교 사례가 없어 이 값의 live 효과는 미확인이다.
- 검색 격자 2/3/5m를 89개 장면에서 순서를 바꿔 세 번 비교했다.
  모든 후보 판정·비용이 같았고 3m의 총 시간 개선은 0.7% 정도이나 p95는
  오히려 느려 5m를 유지했다. 별도의 8/10m 비교도 개선 근거가 없었다.
- 정지 여유 5/4/3m를 정지 구간에서 더 촘촘히 149개 장면으로 비교했다.
  유효 입력 148개에서 실행 가능 후보 없음은 모두 9개, 유한 ETA는 모두
  122개로 동일했다. 정지 감소 근거가 없어 5m를 유지했다.

계획 가속·제동 이득·횡제어 이득·차량 크기·충돌 거리·고정 코스 속도는
추가 변경하지 않았다. 나머지 짧은 장애물 예측 정지는 이번 완화로 해결됐다고
주장하지 않는다. 실제 충돌·완주 시간·정지 시간은 다음 주행에서 확인해야 한다.

## 검증·재현

단위시험은 0.65/0.7초 scene 허용, 0.701초·미래 stamp·무효 scene·epoch
불일치 거부, 위치 입력 0.5초 제한과 비개발 모드 0.5초 제한을 검사한다.
중앙 실행 프로필과 설정값 일치, 원본 stamp 보존도 검사한다.

catkin 빌드, Planner 76 / Controller 173 / Common Messages 40개 테스트,
중앙 공개 계약 29개 테스트와 다이어그램 일치 검사가 통과했다.

원시 자료와 분석 스크립트는
`/home/paik/morai-artifacts/live-param-tuning-20260923/`에 있다:
`baseline.bag`, `planner-before.yaml`, `controller-before.yaml`,
`stale_replay.py`, `stale-replay.json`, `grid_repeat.py`, `grid-repeat.json`,
`stop_margin.py`, `stop-margin.json`, `build-final.log`, `tests-final.log`.
