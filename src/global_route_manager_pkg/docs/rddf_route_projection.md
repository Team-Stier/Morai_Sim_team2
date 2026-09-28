# RDDF 기반 Route Progress 투영 메모

> 이 문서는 RDDF 투영 원리를 기록한 참고 메모다. 사용자 결정에 따라 이번
> Planner Mode Manager 통합에서는 새 투영 알고리즘을 구현하지 않고 기존
> `global_route_manager_pkg`의 `RouteContext.progress`를 그대로 사용한다.

## 목적

Ego 위치를 대회 제공 전역 RDDF에 투영하여 다음 값을 계산한다.

- `route_s`: 전역경로 시작점부터 투영점까지 RDDF를 따라 측정한 누적거리
- `d`: RDDF 진행 방향을 기준으로 한 Ego의 좌우 이격거리
- `segment_index`: 투영점이 속한 RDDF 선분의 인덱스

`route_s`는 Ego가 실제로 주행한 누적거리가 아니다. 장애물을 피해 RDDF에서
벗어나도 코스상의 진행 위치를 나타낸다.

## 권장 투영 방식

Ego 방향에 수직선을 그어 RDDF와 만나는 점을 사용하지 않는다. Ego heading은
회피와 급회전 중 RDDF 방향과 달라질 수 있기 때문이다.

RDDF의 연속된 두 점 `A`, `B`가 만드는 선분에 Ego 위치 `P`를 직교 투영한다.

```text
A ●────────────× Q────────● B
               │
               │ 최단거리 d
               ● P (Ego)
```

```text
t = clamp(((P-A) · (B-A)) / |B-A|², 0, 1)
Q = A + t(B-A)
route_s = cumulative_s[A] + t × |B-A|
```

모든 후보 중 거리, 진행 방향과 이전 진행도의 연속성이 맞는 투영점을 선택한다.

## 이전 index의 의미

`이전 index`는 **전역 RDDF를 구성하는 점열의 이전 투영 선분 인덱스**다.

```text
RDDF points: P0, P1, P2, ... P4430
segments:    P0-P1 = 0, P1-P2 = 1, ...
```

예를 들어 이전 투영점이 `P1200-P1201` 선분에 있었다면 `segment_index=1200`이다.
다음 갱신에서는 이 주변 선분을 우선 검색하여 교차로, 평행 차로와 폐곡선의
시작점·종점 사이에서 다른 구간으로 잘못 점프하는 것을 막는다.

다만 실제 구현에서는 점 간격이 일정하지 않을 수 있으므로 raw index 범위보다
이전 `route_s`를 기준으로 한 거리 범위를 권장한다.

```text
검색 시작 = previous_route_s - backward_search_m
검색 끝   = previous_route_s + forward_search_m
```

예시 범위는 뒤 15 m, 앞 120 m다. 이 값은 실제 Localization 오차, 속도와 호출
주기를 기준으로 검증한 뒤 확정한다.

## 후보 선택 조건

1. 이전 `route_s` 주변의 거리 범위에 있는 RDDF 선분만 후보로 만든다.
2. 각 선분에 Ego 위치를 직교 투영한다.
3. Ego heading과 선분 진행 방향이 반대인 후보를 제외한다.
4. 횡방향 거리와 이전 진행도와의 연속성이 가장 좋은 후보를 선택한다.
5. 작은 후퇴는 Localization 잡음으로 보고 이전 `route_s`를 유지한다.
6. 큰 점프, 큰 횡방향 이탈 또는 Localization `reset_id` 변경은 재매칭 상태로 처리한다.

## 한 번 완주하는 운용

한 실행에서 한 바퀴만 주행하므로 `route_s`는 `0`부터 전체 RDDF 길이까지
증가시킨다. 종점에 도달하면 `FINISHED` 상태로 전환하며 `route_s=0`으로 다시
순환시키지 않는다. 두 번째 주행은 프로그램을 재시작하여 상태를 초기화한다.

폐곡선의 시작점과 종점이 가까워도 이전 진행 이력과 제한된 검색 범위를 사용하므로
출발 직후 종점으로 잘못 매칭하지 않아야 한다.

## Planner 전환에서의 사용

확정된 `route_s`를 구간별 Planner 상태 판단에 사용한다. 실제 전환 구간 값과
Planner 배정은 `path_planning_pkg/config/planner_mode.yaml`에 저장한다.

```text
route_s
  → Planner Mode Manager
      → Z1·Z2·Z3·Z5: Hybrid A*
      → Z4 고주로: Frenet
```

Planner 전환 판단은 `global_route_manager_pkg`가 발행하는 하나의 진행값을
공유하며, 각 Planner가 서로 다른 방식으로 코스 진행도를 다시 정의하지 않는다.
