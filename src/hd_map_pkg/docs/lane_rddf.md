# 코스 주변 추가 차로 RDDF

공식 전역경로 TXT를 보존하고, 고정 KATRI MGeo 지도에서 추가 차로 중심선을
0.5 m 이하 간격의 `x y z` TXT로 추출한다. 파일마다 독립된 연속 구간이며 서로 다른
TXT의 끝과 시작을 연결해서는 안 된다. SIM local ENU / `map`, 단위 m, 원본 높이를
보존한다. 여기서 RDDF는 기존 전역경로와 같은 XYZ 행 형식을 의미한다.
아래 고주로 파생 연결 설정을 적용한 구간은 예외다. 명시한 원본 successor
체인만 차로별로 묶고, 접속부 XYZ는 전역경로에 맞춘 파생 곡선으로 생성한다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun hd_map_pkg hd_map_tool lane-rddf
```

출력은 `data/derived/lane_rddf/manifest.json`과 manifest에 열거된 차로별 TXT다.
manifest는 원본 SHA-256, 전역경로 SHA-256, 적용 임계값, MGeo link/boundary ID,
허용 횡단 위치의 source/target 샘플과 XYZ를 포함한다. 재실행 후에는 반드시
manifest에 열거된 파일만 사용한다. 이전 실행의 TXT가 폴더에 남을 수 있다.
파생물은 Git 대상이 아니며 코드·설정·고정 원본에서 재생성한다.

## 포함 조건

- 코스로부터 XY 15 m 이내, 진행 방향 내적 0.8 이상, 높이 차 1.5 m 이하.
- `opp_traffic` 차로 제외. 가장 가까운 경로 **선분**으로 정합하며 중복점을 무시한다.
- 인접 차로는 MGeo의 명시적 destination과 `can_move_*_lane=true`를 모두 요구한다.
- 양 차로가 공유하는 일반 차선 `503`, 흰색, 단일 `broken`, 통행 제한 없음만 허용한다.
  중앙선·실선·노란선·버스 차로·복합선·알 수 없는 표시는 허용하지 않는다.
- source/target의 진행 방향과 좌우 관계를 확인하고, 횡단 선분이 실제로 통과하는
  양쪽 차로 경계가 정확히 하나이며 허용 점선인지 확인한다.
- 점선 fragment 양 끝에서 3 m를 제외한다. 실선으로 바뀌는 지점을 넘겨 허용하지 않는다.
- 진행 방향의 차로 내 연결과 명시적 successor(끝점 간격 0.5 m 이하), 허용 횡단으로
  그래프를 만든다. 전역경로 대응 샘플에서 도달하고 전역경로로 돌아올 수 있는
  샘플만 남긴다. 샘플 간격이 끊기면 출력 TXT도 분리한다.
- 기존 전역경로에서 0.8 m 이하인 샘플은 추가 RDDF에서 제외한다.

수치는 `config/map_conversion.yaml#lane_rddf`에 있다. 보수적인 정적 후보 추출이므로
전체 법적 허용 차로를 완전히 열거했다고 보장하지 않는다. 다른 차선 코드·복합선의
방향별 허용 여부는 근거가 확인되기 전까지 포함하지 않는다. 최근 공식 웹 규정은
이번 작업에서 접근 실패했으며 저장소의 규정 베이스라인과 고정 원본 속성을 사용했다.

## RViz

기존 HD Map 내부 MarkerArray에 다음 namespace를 추가한다. 공개 topic/메시지와 TF는
바꾸지 않는다. RViz의 **HD Map → Namespaces**에서 개별 표시를 끌 수 있다.

| 색 | Namespace | 의미 |
|---|---|---|
| 초록 | `global_route` | 기존 공식 전역경로 |
| 하늘색 | `lane_rddf` | 추가 차로 중심선 |
| 주황 | `lane_change_windows` | 허용 점선 횡단 위치, 약 10 m마다 표시 |

주황색 횡단선은 조향 가능한 차선변경 궤적이 아니다. 차로 중심선도 신호·NPC 간격·
장애물·체크포인트 순서·차체 swept volume을 검증한 주행 명령이 아니다. 이 판단과
곡률을 만족하는 차선변경 궤적 생성은 Planner의 책임이다.

## 2026-09-21 검증

- 고정 원본에서 추가 연속 차로 16개, 허용 횡단 샘플 3,412개 추출.
- RViz 전송: `lane_rddf` LINE_LIST 4,484점, `lane_change_windows` 336점,
  모두 `map`. 실행 중 `vehicle_rviz` 구독 연결과 실제 ROS callback 확인.
- HD Map 단위 테스트 47개 통과. 지도 표시 3개 및 producer-consumer 계약 2개 통과.
- catkin 대상 패키지 빌드 및 중앙 다이어그램 `--check` 통과.
- 전체 visualization 회귀에는 upstream `origin/main`의 WorldModel 메시지 생성 누락과
  whitelist 빌드에서 제외한 LiDAR 실행파일 부재가 있어 실패했다. 지도 변경의
  성공으로 전체 launch 성공을 주장하지 않는다. MORAI closed-loop는 미검증.
- 사용 중인 기존 worktree는 보존했다. 현재 RViz에는 별도 read-only preview 프로세스로
  두 namespace를 추가했다. 이 프로세스는 `/lane_rddf_preview`이며
  `rosnode kill /lane_rddf_preview`로 중지할 수 있다. 원래 worktree에서 재시작하는
  visualization에는 이번 소스 변경이 아직 포함되지 않는다.

### 교차로 진입 유지 수정

사용자가 현재 위치 `map` 약 (-95.916, 291.563)에서 교차로 진입이 필수임을 지정했다.
우회 link `A2256W000739`와 그 연속 link `A2256W000849`를
`lane_rddf.excluded_link_ids`로 제외한다. 원본 지도나 공식 전역경로는 수정하지 않는다.
제외는 그래프 작성 이전에 적용하므로 route seed·successor·차선변경 연결에도 적용된다.
이 목록은 사용자 지정 코스 제약이며 해당 도로의 일반적인 교통법상 통행 금지를 뜻하지 않는다.

재생성 결과는 추가 RDDF 14개, 횡단 샘플 3,412개다. 제외한 두 TXT도 파생 폴더와
배포 ZIP에서 제거했다. 현재 RViz의 실제 수신 마커가 14개 RDDF의 좌표와 정확히
일치함을 확인했다. HD Map 테스트 48개, 표시 3개, 계약 2개, catkin 빌드와
다이어그램 검사가 통과했다. 앞서 기록한 전체 시각화 회귀의 upstream 제약은 그대로다.

고정 코스 속도 정책 적용: 고주로는 분홍색, 일반 추가 RDDF는 하늘색이다. manifest의 `speed_limit_kph` 배열은 XYZ 각 점의 제한을 `null`(제한 없음) 또는 `58`으로 기록한다. `speed_sections`는 구간 색상 표시용이며 원본 XYZ를 변경하지 않는다.

## 고주로 분기·합류 연결 (2026-09-22)

기존 추출 결과는 고주로 진입 약 148 m 뒤부터 추가 차로가 남고, 각 차로가
4개 파일로 나뉘었다. 사용자 요청에 따라 다음 명시적 MGeo successor 체인을
각각 하나의 연속 RDDF로 만든다. 일반 구간 추가 RDDF 6개는 그대로 유지한다.

| 파생 RDDF | 원본 링크 체인 (A2256W 접두사 생략) |
|---|---|
| `high_speed_inner` | 000410 → 000430 → 000434 → 000422 |
| `high_speed_outer` | 000418 → 000431 → 000435 → 000423 |

두 RDDF는 `A2256W000411` 시작점의 전역경로 위치
약 `(62.074, 256.495)`에서 함께 분기한다. 80 m에 걸쳐 각 원본 차로로
접속하고, 마지막 45 m에서 전역경로에 합류한다. 합류점은 추가 차로 원본의
종점 `A2256W000423` 끝, 약 `(75.414, -217.855)`이다.
고주로 속도 정책의 끝 `(70.214, -365.748)`까지는 합류한 전역경로를 따른다.

접속부는 위치·접선과 양 끝의 0 이차미분을 맞추는 5차 곡선이다. 중간 구간은
원본 차로 중심선을 보간한다. XYZ 샘플 간격 설정은 0.5 m이며 `route_s`는
전역경로 진행방향으로 증가한다. 공식 전역경로 TXT와 MGeo 원본은 수정하지 않는다.

`source_link_ids`는 합성 차로의 원본 링크 순서이고 `link_id`는 첫 링크다.
합성 차로의 `source_indices`는 파생 TXT의 행 인덱스다. 원본 링크의 샘플
인덱스로 해석하지 않는다. `geometry_origin=derived_course_connection`으로
출처를 구분하고 분기·합류를 graph에 기록한다. 이 연결 형상은 새로운 점선
횡단 허가를 뜻하지 않는다. 기존 허용 창은 원본 형상이 유지되는 구간만
남겨 새 차로 ID·누적 거리로 변환하며 금지 경계는 보존한다.

HD Map 런타임, XYZ 내보내기, HTML과 RViz가 같은 생성 함수를 사용한다.
실행 중인 노드의 캐시는 자동 갱신되지 않으므로 새 지도를 사용하려면 해당
지도·시각화 실행을 다시 시작해야 한다. MORAI closed-loop 주행 검증은 별도다.

검증: HD Map 테스트 54개, Planner/지도 전달 테스트 42개, 중앙 계약 테스트
46개와 다이어그램 검사를 통과했다. catkin 빌드와 HD Map catkin 테스트도
통과했다. 실제 생성 지도는 ROS 메시지 직렬화 후 Planner의 일반/형상 전용
모드 모두에서 읽혔다. 추가 RDDF는 8개, 기존 점선 허용 창은 22개이며,
고주로 두 RDDF의 최대 점 간격은 각각 0.49995 m와 0.49998 m다.
공식 전역경로·지도 원본 해시와 나머지 6개 RDDF 형상이 유지됨을 확인했다.
