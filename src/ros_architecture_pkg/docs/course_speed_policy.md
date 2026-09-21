# 고정 코스 속도와 RDDF 표시

2026-09-21 사용자 지시: 고주로만 속도 제한 해제, 그 외 최대 50 km/h 고정,
고주로 목표 순항 100 km/h. 원본 규정/지도 파일을 수정하지 않는다.
원본은 [course_speed_policy.yaml](../config/map/course_speed_policy.yaml)이다.

고주로는 `A2256W000411` 첫 점부터 `A2256W000153` 마지막 점 직전까지다.
원본 경로의 0-based index 2275~3529 경계이며 실행 시 중복 제거 후 좌표로
다시 찾는다. YAML 경계와 MGeo 변환 좌표의 차이는 각각 0.000000532 m,
0.000000171 m다. 끝점부터 일반 구간 제한을 적용한다.

`hd_map_pkg.course_speed`의 동일 정적 구간 판정으로 planner, RViz/HTML과
RDDF를 연결한다. RDDF `speed_limit_kph` 배열은 각 XYZ 점과 대응하고 고주로는
`null`, 그 외는 `50`이다. `null`은 제한 해제이며 목표 100과 다르다.
`speed_sections`는 표시용 연속 edge 구간이다. 원본 XYZ/MGeo max_speed는 보존한다.

Planner는 일반 목표 48 km/h, 고주로 목표 100 km/h를 사용한다.
곡률에 따른 횡가속도 2 m/s²와 종감속도 2 m/s²의 역방향 속도 프로파일을
폐경로 전체에 계산하고, 감속에 15 m 선행 거리를 적용한다. 곡선과 감속
구간에서는 목표 순항보다 낮게 주행한다. 뒤쪽 보조 경로점에는 현재 구간
속도를 넣어 출구에서 과거 고주로 속도가 Controller에 전달되지 않게 한다.
제어 코어 원본은 변경하지 않았다. 이 실행의 Pure Pursuit lookahead는
`3 + 0.6 × 속도(m/s)`, 최대 30 m다. 별도 방어 계층은 추가하지 않았다.

고주로 전역경로/추가 RDDF는 분홍색, 일반 전역경로는 초록색,
일반 추가 RDDF는 하늘색이다. RDDF 14개·횡단 위치 3,412개와 필수 교차로
우회 제외 조건을 유지했다. 추가 RDDF 중 고주로 점은 1,237개다.

검증: catkin 빌드, map/planner/controller/중앙 계약/bringup의
`catkin_test_results` 집계 314개 결과 항목과 지도 표시·I/O 6개 검사가 통과했다.
경로 hash·경계, 원본 MGeo 좌표, 일반 상한·고주로 목표, 폐경로 감속,
map→odom 변환과 출구의 제어기 입력 속도를 검사했다. 중앙 다이어그램과
render manifest 검사를 통과했고 HTML은 Chromium에서 오류와 색상을 확인했다.

실제 MORAI에서 허용된 GPS/IMU 기반 Localization으로 폐경로를 계속 추종했다.
고주로 목표 100 km/h에 대해 약 99.4 km/h, 일반 구간 최고 48.4 km/h 미만을
관측했다. 출구 전 감속 후 일반 구간을 50 km/h 이하로 통과했다.
선택된 제어기 기준 경로 오차 최대는 관측 구간에서 약 0.62 m였다.
이는 해당 실행 표본이며 교통신호·장애물 회피나 모든 조건의 검증은 아니다.

산출물: `/home/paik/morai-artifacts/course-speed-20260921/`.
주행 기록: `/home/paik/morai-artifacts/course-speed-live-observation.jsonl`.
실행법과 개발용 Safety 단순 전달 범위는 [전역경로 실행](global_path_demo.md)에 있다.
