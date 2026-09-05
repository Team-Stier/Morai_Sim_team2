# KATRI lane-safe Hybrid A* 설계

## 기준과 좌표

- ROS1 Noetic/catkin을 대상으로 한다.
- `map`은 MORAI scene-local ENU이며 x=east, y=north, yaw 0은 east, CCW가 양수다.
- planning pose와 `base_link` 원점은 rear-wheel midpoint다.
- 차량 envelope는 공식 2023 IONIQ 5 제원(4.635 × 1.892 m, wheelbase 3.0 m, 최소 회전반경 5.87 m)을 사용한다.
- 지도 geometry는 `hd_map_pkg`의 MGeo 3.0→Lanelet2 materialization과 동일한 source-boundary clipping/합성 규칙을 사용한다.

## 탐색

AMET2026 Team Stier 구현의 연속 pose + 이산 discovered set, exact constant-curvature bicycle propagation, OPEN/parent bookkeeping 개념을 독립적으로 재구현했다. 상태 key에는 조향 변화 비용과 일관되도록 `(x, y, yaw, steering bin)`을 넣는다. 각 primitive는 설정 간격의 pose를 출력하며 샘플 사이 rigid-body displacement bound도 재귀 검증한다. 일반 주행은 대회 제공 4,430점 전역경로의 `RouteContext` 진행도 기반 전방 slice를, 차선 변경·인접 차로 follow와 안전 fallback은 검증된 MGeo branch 중심선을 guide로 쓴다. guide는 heuristic·조향 후보 순서와 누적 횡방 cost에 영향을 주지만 안전 판정 권한은 없다. reference 밖 경로도 hard-wall 검사를 통과하면 후보가 될 수 있고, reference 위 경로도 벽 검사에 실패하면 거부한다. 제공 경로는 폐곡선이므로 종료선 직전의 짧은 로컬 slice는 `L→0`을 넘을 수 있지만, `RouteContext` 진행도가 1 lap 종료 `L`에서 고정되는 현재 계약을 planner가 다중 lap 진행도로 변조하지는 않는다. 공식 goal footprint가 invalid이면 설정 local horizon 안의 더 짧은 공식 goal만 검사하고, 모두 invalid이면 같은 hard corridor의 기존 safe MGeo goal로 fallback한다.

색인 node 수 상한과 별도로 wall-clock 탐색 상한과 primitive당 collision-evaluation 상한을 두어 적대적 goal이나 폐쇄 공간에서도 유효기간 밖으로 무한정 block하지 않는다. 탐색 중 ego가 이동했으면 최신 pose와 collision-certified dense path 사이 위치·yaw 추종오차와 남은 horizon을 gate로 검사한다. 통과해도 snapshot 시각부터 연속 검증된 원래 path 전체와 원래 `s`를 유지한다. suffix를 잘라 최신 ego→anchor 연결을 암묵적으로 만들지 않는다. controller의 nearest-point 선택, planning latency 보상과 실제 추종 궤적은 이 planner의 충돌 증명 범위 밖이다.

전역 MGeo 전체를 5 cm grid로 만들지 않는다. Global Route Manager가 고른 Link와 선택 successor/adjacent Link만 corridor로 materialize한다. 교차로 shortcut 방지는 다른 outgoing Link를 corridor에 포함하지 않는 route topology selection으로 수행한다.

참조 저장소에는 root LICENSE/NOTICE가 없고 package metadata에만 Apache-2.0 표기가 있어 source를 복사하지 않았다. 배포 전 원 저작권자 라이선스 의사를 별도로 확인해야 한다.

## 벽 상태

| 상태 | 주행 polygon | 열리는 경계 | 실패 동작 |
|---|---|---|---|
| keep route | 선택 predecessor/current/successor | topology가 검증된 종방향 seam만 | 모든 횡방향 경계 hard |
| turn connector | 선택한 predecessor→connector→successor | 두 인접 polygon의 검증된 virtual longitudinal mouth만 | lateral marking·terminal end cap은 hard, 다른 교차로 branch는 polygon에서 제외 |
| highway overtake | current + 승인된 adjacent | 두 차로가 공유하는 실제 pure-dashed 선 하나 | 조건 하나라도 누락되면 벽 유지/no-path |
| crossing latched | 최초 승인과 같은 두 차로 | 횡단 완료까지 같은 공유선 | solid/unknown/외측 벽은 latch로도 열 수 없음 |

고주로 최초 개방 조건은 모두 AND다.

1. route context가 공식 `A2256W000411` 시작~`A2256W000153` 끝 구간 안이다.
2. same-link 선행차 observation이 fresh, finite, confidence 기준 이상이다.
3. forward bumper gap이 `0 <= gap <= 10.0 m`다.
4. MGeo directional adjacency와 whitelist가 일치한다.
5. 양 lane이 같은 source boundary를 공유하고 그 적용 구간이 pure dashed다.
6. 남은 측정 dashed 길이 안에서 maneuver를 끝낼 수 있다.

최초 valid plan 뒤에는 crossing 상태를 latch한다. 선행차 gap이 바뀌었다는 이유로 차량 아래에 벽을 즉시 다시 만들지 않는다. 다만 crossing 중 lead observation 자체가 stale/invalid가 되면 현재 baseline은 불확실한 장애물을 지우지 않고 stop-required를 낸다. target centerline 정착 후 adjacent branch를 따라가며 route merge Link에서 원 route로 복귀한다. crossing/follow/merge 상태 전이는 계획 중에 먼저 바꾸지 않고, 최신 pose로 조건을 다시 확인한 valid trajectory 발행이 성공한 뒤에만 commit한다.

## KATRI에서 확인된 정책

- 조건부 활성: `A2256W000420 → A2256W000430` (`B2256W000034` pure dashed)
- 조건부 활성: `A2256W000408 → A2256W000434` (`B2256W000044`의 실제 측정 dashed overlap만)
- 비활성: `A2256W000411 → A2256W000409`; `B2256W000038`은 mixed solid/dashed다.
- 비활성: `A2256W000445 → A2256W000422`; shared source boundary가 없어 합성선을 지울 근거가 없다.
- 비활성: `A2256W000153 → A2256W000432`; 지도 자체가 lane change 불가이며 solid 구간이 포함된다.
- 향후 좌회전 준비 후보 `A2256W000309 → A2256W000308`은 현재 비고주로 정책에서 닫혀 있다.

중요하게, 제공 global path의 `A2256W000411` centerline은 mixed 경계 `B2256W000038`을 global progress 약 1246.4 m에서 교차한다. 현재 요구처럼 solid 성분을 절대 열지 않으면 그 위치는 `no-path/stop-required`가 맞다. 완주용 mandatory merge 예외는 실제 scene 확인과 사용자 승인 없이 추가하지 않는다.

## Clearance와 보증 경계

route 지도 분석에서 A411 모순을 제외한 최소 center-on-route 잔여 폭은 약 0.426 m였다. 경계 centerline으로부터의 초기 hard clearance는 `0.075 m` 선 반폭 + `0.020 m` bounded map simplification 오차 + `0.200 m` 운용 여유 = `0.295 m`다. 선 contact는 무조건 invalid다. wheel-track 공식값이 없어 body width를 wheel contact proxy 폭으로 사용해 보수화한다. 규정 판정이 wheel 기준이므로 lane wall은 이 네 wheel proxy의 연속 궤적을 제한한다. 차체 overhang은 회전 중 차선 도색 위를 지날 수 있지만, 동적 객체와의 충돌에는 전체 차체 직사각형을 사용한다.

다만 0.200 m 운용 여유는 Localization covariance, controller cross-track error와 tire contact patch를 모두 증명한 총 오차 예산이 아니다. 실제 MORAI closed-loop에서 다음을 확인하기 전 “절대 차선을 밟지 않는다”는 물리 보증으로 표현하면 안 된다.

- 실제 scene map 원점/버전 일치
- odometry covariance 및 blackout drift 상한
- controller가 planner와 동일한 원호/보간을 따르는지
- 최대 cross-track/yaw tracking error
- 실제 wheel track/tire 폭과 line physical width
- target-lane front/rear object gap을 포함한 World Model 안전성

## Fail-closed

wrong frame, zero/future/stale timestamp, NaN/Inf(속도·두 covariance 포함), route-global-path 불일치, 지도 topology 불일치, 조건 미충족, 시작/목표 footprint invalid, search node/time limit, latest-ego/path 비정렬, no-path는 빈 visualization path와 만료된 non-valid trajectory를 낸다. 탐색이 끝난 뒤 최신 odometry/route/lead와 경로를 다시 검사하며, 유효기간은 탐색 완료 시각이 아니라 입력 snapshot을 수락한 시작 시각부터 센다. 최종 serialization 직전에도 deadline과 입력 freshness를 다시 확인하며 이전 valid trajectory는 유지하지 않는다.

`/world_model/lead_vehicle`가 아예 없거나 malformed/future/stale, odometry와 허용 시각차 초과, 현재 Link 불일치이면 직진 중에도 정지한다. 객체가 없는 정상 상태는 World Model이 fresh, finite `valid=false` heartbeat로 명시해야 한다. 이는 조용하거나 정렬되지 않은 topic을 빈 도로로 오인하지 않기 위한 중앙 계약이다.

Hybrid A* 탐색 주기는 2 Hz지만 별도의 50 Hz input watchdog이 이미 발행된 valid trajectory가 있는 동안 odometry, route, lead와 global path를 다시 검사한다. 입력 callback도 새 snapshot이 invalid이면 즉시 non-VALID trajectory와 빈 local path를 발행하고, 만료는 20 ms 이내의 다음 watchdog tick에서 검출한다. invalid snapshot마다 fault revision을 올리고 최종 revision 확인과 VALID 발행을 callback/watchdog/failure 발행과 같은 output lock에서 처리하므로, 오래된 탐색 결과가 더 최신 failure를 덮어쓸 수 없다. 정상적인 새 lead/odometry/route가 일반 post-search 검증 뒤 도착한 경우에도 같은 lock 안에서 최신 ego 정렬, corridor policy, 장애물과 전체 path를 다시 검사한다. 이 잠금 검사는 한 watchdog period인 20 ms를 넘기면 성공시키지 않는다. 노드 자체가 죽거나 ROS transport가 단절된 경우에는 downstream이 `valid_until`과 0.75 s topic timeout으로 정지해야 한다.
