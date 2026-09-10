# MGeo 3.0 → Lanelet2 schema mapping

| 요구 layer | MGeo source | Lanelet2 output |
|---|---|---|
| 차선 중심선 | `link_set.points` | semantic segment별 way `type=virtual, subtype=centerline`; lanelet member `centerline` |
| 좌·우 경계 | `link.lane_mark_left/right`, `lane_boundary_set` | 감사용 source way + segment chainage에 맞춰 자른 lanelet `left/right` way |
| 선 종류·색 | `lane_type`, `lane_shape`, `lane_color` | `type`, `subtype`, `color`, `lane_change`, 원본 `mgeo:*` tag |
| 누락 경계 | link centerline, `width_start/end` | `type=virtual, subtype=synthetic_boundary, mgeo:synthetic=yes` |
| 정지선 | boundary `lane_type=[530]` | way `type=stop_line`; 교차하는 접근 신호 regulation의 `ref_line` |
| 노면 화살표·과속방지턱 | `surface_marking_set` | source ID당 하나의 `multipolygon`; 연결 link에서 centroid가 속한 lanelet segment tag |
| 보행 횡단보도 | single crossing `sign_type=5321/533` | `subtype=crosswalk`, `participant:pedestrian=yes`, `one_way=no`; 533은 `mgeo:raised=yes` |
| 자전거 횡단도 | single crossing `sign_type=534` | `subtype=bicycle_crossing`, `participant:bicycle=yes`, `one_way=no` |
| 오르막 표시 | single crossing `sign_type=544` | `road_marking/uphill_slope_marking`; crossing participant tag 없음 |
| 신호등 위치·ID | `traffic_light_set.point/idx` | way `type=traffic_light`, 표준 signal subtype, `mgeo:id` |
| 차량 신호-차로 | synced group, signal node, incoming link | stop-line group별 `traffic_light` regulatory relation; source link의 마지막 lanelet segment가 참조 |
| 보행 신호-crossing | crosswalk group/reference | `traffic_light` regulatory relation; crossing area가 참조 |
| 신호 controller | controller set/data | `mgeo:controller_id`; phase/state는 static rule로 쓰지 않음 |
| 교차로 | `junction_set.road_id_list` | `mgeo:derived=yes`인 convex-hull `intersection_area` multipolygon |
| 선행·후행 | link `from_node_idx/to_node_idx` | 안전한 경우 공유 bound endpoint Point IDs; 항상 explicit routing JSON v2에 보존 |
| 측방 연결 | lateral destination, `can_move_*` | 완전히 동일한 bound만 공유; source 관계는 routing JSON에 보존 |
| 제한속도 | `link.max_speed` | 모든 lanelet segment에 `speed_limit="N km/h"` |
| 진행방향 | `link.related_signal`, surface arrow | `turn_direction`; U-turn은 `mgeo:maneuver` |

## Source link 1:N lanelet 분할

한 source link의 좌·우 boundary fragment 시작·끝을 centerline chainage로 투영하고,
KATRI 접점 투영 jitter 범위인 0.5 m 안의 event를 합친 뒤 의미가 일정한 구간마다
lanelet relation을 만든다. 따라서
`mgeo:id`는 source link ID라서 여러 relation에 반복되는 것이 정상이다. 각 relation은
다음 tag로 독립 식별되고 source 전체 길이를 빈틈없이 덮는다.

- `mgeo:segment_id=<source-link>#<zero-based-index>`
- `mgeo:segment_index`, `mgeo:segment_count`
- `mgeo:start_chainage_m`, `mgeo:end_chainage_m`

현재 고정 스냅샷의 1,317개 source link는 2,346개 lanelet relation이 되며, 그중 559개
link가 두 개 이상으로 분할된다. ID map의 `lanelet_relations`는 source link별 relation ID
배열이고 `lanelet_segments`는 segment ID별 relation ID다.

원본 boundary way는 source feature 추적용이며 lanelet relation이 직접 참조하는 것은
구간에 맞춰 자른 derived way다. 두 lanelet의 lateral bound는 source boundary ID,
`type/subtype/color`, 잘린 전체 geometry, endpoint identity가 모두 같을 때만 동일 OSM
way를 재사용한다. 부분 overlap이나 근접 geometry는 강제로 공유하지 않는다. 따라서
`can_move_*`가 있어도 공유 way가 없으면 native Lanelet2 lane change로 간주할 수 없고
`KATRI_routing_graph.json`의 source 관계를 읽어야 한다.

## 보존되는 link 필드

주요 구조 필드 `idx`, `from_node_idx`, `to_node_idx`, `road_id`, `ego_lane`,
`lane_mark_left/right`, `left/right_lane_change_dst_link_idx`, `can_move_left/right_lane`,
`max_speed`, `related_signal`은 OSM tag, primitive membership, routing graph 또는 ID map에
보존된다. 원본 전체 JSON은 submodule에 그대로 있으므로 손실 필드는 source ID로
역참조할 수 있다.

노면 표시 하나는 source ID당 정확히 하나의 area relation과 outer way를 갖는다. area
relation에만 `area=yes`를 두고 outer way는 LineString으로 유지한다. `link_id_list`의 각
link에 대해 표시 centroid chainage를 포함하거나 가장 가까운 lanelet segment 하나에
`mgeo:surface_markings`를 기록한다.

한 traffic-light head가 서로 다른 stop line의 접근 차로를 제어하면 signal way는 공유하고
stop-line group별 regulatory relation을 만든다. 각 relation은
`mgeo:regulatory_instance=<signal>#<index>`로 식별하며 `ref_line`은 최대 하나만 갖는다.

## 보존되는 boundary 필드

`idx`, `lane_type`, `lane_shape`, `lane_color`와 기능 분류는 각 non-degenerate source
boundary way에 저장한다. `lane_width`, dash interval과 polynomial coefficient는 원본
geometry 재구성 metadata로 원본 JSON에 유지하고 현재 OSM styling에는 직접 쓰지
않는다.

XY distinct point가 하나뿐인 source boundary 5개는 OSM way의 최소 조건을 만족하지
않아 생략한다. validator는 나머지 2,294개 source boundary의 1:1 coverage를 검사하고
생략 ID는 별도 warning으로 보고한다.

## 파생·fallback과 비표준 tag

`mgeo:*`, `source:*`, `simulator:*`는 provenance와 추적을 위한 custom tag다.
`turn_direction`, `intersection_area`는 Autoware 계열 확장이며 core Lanelet2의 필수
tag가 아니다. lanelet relation의 표준 member 역할은 `left`, `right`, `centerline`,
`regulatory_element`만 사용한다.

`KATRI_routing_graph.json` v2는 source link graph와 모든 lanelet segment의
predecessor/successor를 함께 기록한다. OSM에서 안전하게 endpoint를 공유하지 못한
연결의 fallback이지만 Lanelet2 표준 파일 자체는 아니므로 consumer 통합이 필요하다.
교차로 polygon도 원본 polygon이 아니라 junction membership의 convex hull이다. 넓이와
bbox를 tag로 남기며 120 m를 넘는 broad hull은 validator warning이지 정밀 drivable
intersection 경계가 아니다.

공유 endpoint를 만들 때 생기는 위치 보정은 한 vertex에 꺾어 붙이지 않고 기본 10 m
구간에 smoothstep으로 분산한다. topology endpoint 자체는 정확히 일치하며 원본 boundary
추적용 way는 이 보정의 대상이 아니다.

전역경로 TXT는 이미 SIM local ENU 좌표이므로 OSM primitive로 변환하지 않는다. HTML
preview의 `globalRoute` 배열에 4,430개 XY를 원본 순서·중복 그대로 넣어 초록색 비교
overlay로만 렌더링한다.
