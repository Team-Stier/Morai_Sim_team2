# 코스 신호 정지선 계약

중앙 `messages/rddf_planning_messages.yaml` v0.4.0과 `interface_contract.yaml`의
`HdMap.signal_stop_lines` / `SignalStopLine`이 원본 계약이다. 추가 공개 토픽은 없다.

HD Map은 원본 MGeo의 차량 신호 → 접근 차로 연결을 사용한다. 기존 Lanelet2
exporter와 같은 규칙으로 접근 차로 마지막 40 m의 `530` 정지선을 찾으며,
교차 허용 오차는 0.75 m다. 기준 값은 `hd_map_pkg/config/map_conversion.yaml`이다.
공식 전역경로와 정지선 부근에서 방향·위치가 맞는 접근 차로 또는 승인된 추가
RDDF의 접근 차로만 선택한다. 반대 방향·보행 신호와 제외 링크는 포함하지 않는다.
같은 정지선을 사용하는 여러 차로·신호는 원본 정지선 ID 하나로 합친다.

각 `SignalStopLine`은 원본 정지선 ID, 변환된 원본 XYZ 점 배열, 접근 링크 ID와
신호등 ID 배열을 보존한다. 좌표는 부모 HdMap의 `map` 프레임, 단위 m이며
부모의 정적 지도 버전·load stamp에 속한다. 정지선 ID 간 가상 연결은 만들지 않는다.
신호의 동적 색상·phase는 정적 지도에 넣지 않는다.

현재 고정 KATRI 지도에서 선택되는 정지선은 5개다.

| 원본 정지선 ID | 접근 링크 (A2256W 접두사 생략) | 연결 차량 신호 (C1256W 접두사 생략) |
|---|---|---|
| B2256W000235 | 000728 | 000016, 000018 |
| B2256W000252 | 000308, 000309 | 000052 |
| B2256W000266 | 000310, 000315 | 000123 |
| B2256W000447 | 000213 | 000091 |
| B2256W000556 | 000219 | 000077 |

RViz의 기존 `HD Map` display는 같은 추출 함수를 통해 정지선을 빨간 횡선과
`STOP 1`~`STOP 5` 글자로 표시한다. 번호는 원본 ID 정렬 순서의 표시 번호이며
경로 미션 순서가 아니다. namespace는 `signal_stop_lines`와
`signal_stop_line_labels`다. 화면에서만 높이를 평면으로 투영한다.
표시만으로 정지 명령이나 신호 판단을 추가하지 않는다.

## 생산자·소비자 영향

- `common_msgs_pkg`: 중앙 스키마대로 새 타입과 HdMap 필드를 생성한다.
- `hd_map_server_node`: 같은 source 검증 후 정지선을 지도 내용과 map_id에 포함한다.
- 기존 Localization, Route, World Model, Planner: 새 정지선 필드는 정적 정보이며
  기존 제어 동작은 유지한다. HdMap MD5가 바뀌므로 구독 노드는 재빌드·재시작한다.
- Visualization: 공개 구독을 추가하지 않고 기존 읽기 전용 지도 표시에서 공유
  추출 함수를 사용한다. 지도 source와 공식 경로는 HD Map과 같다.
- 전체 실행 조합은 기존 `system_bringup_pkg` launch를 사용한다.

실행 중인 HdMap producer와 실제 subscriber는 함께 갱신해야 한다. 이전 MD5의
node와 새 MD5의 node를 혼용하면 ROS가 연결을 거부한다. 기존 launch 이름·topic,
TF와 timestamp 정책은 유지한다.

## 검증 (2026-10-06, paik)

HD Map 단위 테스트 62개, 공통 메시지·계약 테스트 40개, Visualization 단위
테스트 22개와 격리된 ROS master의 차량 표시 rostest 8개를 통과했다.
정지선이 포함된 지도 메시지를 실제 producer에서 직렬화하고 Planner의 두 지도
입력 모드로 전달하는 회귀 테스트도 통과했다. 반대 방향·보행 신호 제외,
원본 좌표 보존, 공유 정지선 중복 제거와 분리된 빨간 횡선 표시를 검사했다.
기존 선택된 6개 패키지의 catkin 빌드, XML/YAML과 중앙 그림 검사도 통과했다.
공개 입출력 그림 내용·SVG·PNG는 동일하며 중앙 계약 변경 해시만 갱신했다.

실행 중인 HD Map, 실제 지도 구독 노드인 Route Manager·Planner와 Visualizer를
함께 재시작했다. 공개 지도에 정지선 5개가 원본 변환 좌표·연결 ID와 정확히
일치하고 RouteContext의 map_id가 같으며 Planner status가 다시 발행됨을 확인했다.
RViz HD Map 마커에는 빨간 정지선과 번호 5개가 포함되고 기존 RDDF 11개와
체크포인트 15개가 유지된다. 실제 신호 상태 판단·정지 동작은 이 변경의 검증 범위가 아니다.

산출물: `/home/paik/morai-artifacts/signal-stop-lines-20261006/`의
`stop_lines.json`, `live_verification.json`, `build.log`, `visualization_rostest.log`.
