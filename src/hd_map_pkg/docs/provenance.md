# KATRI HD Map provenance and confidence

## 판정

이 데이터는 실행 중인 `r_kr_pr_k-city_2025.scene`과 강하게 정합하는 **immutable
candidate**다. MORAI 내부 HD Map과 byte-identical한 공식 export라고 단정하지 않는다.

## 원본

| 항목 | 값 |
|---|---|
| 조직 | MORAI-Autonomous |
| 저장소 | `verdict-sdk` |
| commit | `fe27e92166513ac0c6c70a16d85212fc5f833fd3` |
| KATRI tree | `b69c55ce910bfd4e17d709312d7ec47fde9d798e` |
| 저장 시각 | `2026-06-10 06:26:02.333268` UTC |
| 형식 | MGeo 3.0 |
| 좌표계 | WGS84 / UTM zone 52N |
| 원점 | `[305390, 4122845, 0]` |
| metadata license | `MORAI` |

공식 repository의 [KATRI 디렉터리](https://github.com/MORAI-Autonomous/verdict-sdk/tree/fe27e92166513ac0c6c70a16d85212fc5f833fd3/map-data/KATRI)는
HD-map JSON을 공개하지만 저장소 루트의 별도 LICENSE는 확인되지 않았다. 공개 열람과
재배포 허가는 동일하지 않으므로 submodule pointer만 저장한다.

원본 고정은 서로 다른 세 층으로 확인한다.

- superproject의 submodule과 설정은 위 commit 및 KATRI tree를 가리킨다.
- validator는 실제 checkout의 `HEAD`, `HEAD:map-data/KATRI` tree와 KATRI 경로의
  dirty 여부를 설정값과 비교한다. 하나라도 다르면 `immutable_source_checkout`은 fail이다.
- source manifest는 변환 시 실제로 읽은 파일의 raw/CRLF SHA-256과 dedupe alias를 감사
  기록으로 남긴다. 기존 서명과 대조하는 별도의 신뢰 anchor는 아니다.

Git commit/tree는 선택한 공개 저장소 snapshot의 재현성을 제공한다. 이것만으로 실행 중인
SIM 내부 데이터와의 byte 동일성, MORAI의 배포 승인, 지도 의미의 정확성 또는 파생 OSM의
동일성을 증명하지 않는다. 파생물을 재현하려면 이 원본 pin뿐 아니라 동일한 converter
코드와 config revision도 필요하다.

## SIM 정합 근거

예제 scene의 원점은 `[302595, 4124145, 0]`, 좌표계는 UTM52N/ENU다. 따라서 후보
MGeo와 scene local 좌표의 평행이동은 `[+2795, -1300, 0]`이다.

제공 global path 4,430개 점을 이 변환으로 KATRI link centerline과 비교한다. 정확한
수치는 매 빌드의 `KATRI_validation_report.json`에 기록한다. KATRI에는 대회 규정에서
지목한 link `A2256W000411`, `A2256W000153`도 존재한다.

다만 공개 repository 전체에서 scene 문자열 자체가 발견되지 않았고 simulator 내부
gRPC map export도 성공하지 않았으므로, 이것이 runtime map과 byte-identical하다는
주장은 보류한다.

## 원본 hash 주의

`global_info.mgeo_file_hash`의 18개 선언값 중 다수는 repository의 LF 파일을 CRLF로
복원하면 일치한다. `intersection_controller_data.json` 하나는 raw와 CRLF 양쪽 모두
일치하지 않는다. 현재 snapshot의 결과는 raw 일치 2개, CRLF 일치 15개, 불일치 1개다.
파이프라인은 이를 숨기지 않고 warning으로 기록한다. 따라서 upstream 선언 hash를 현재
checkout의 단독 무결성 근거로 쓰지 않고, 실제 commit/tree/dirty 검사를 immutable gate로
사용한다. manifest의 raw SHA-256은 각 빌드가 소비한 byte를 추적하는 감사 기록이다.

## 현재 변환·검증 상태

현재 고정 원본과 converter에서 확인한 주요 coverage는 다음과 같다. 신호 규제 relation은
한 signal head가 여러 stop line과 연결될 때 여러 relation으로 나뉠 수 있으므로, relation
개수를 source signal 개수와 1:1이라고 가정하지 않는다.

| 항목 | 결과 |
|---|---|
| source link → lanelet segment | 1,317개 → 2,346개; 559개 link가 1:N 분할 |
| source boundary | non-degenerate 2,294개 전부 export; degenerate 5개 생략 warning |
| surface marking | source 495개 각각 하나의 area 및 연결 source link별 segment association |
| crossing | 98개: 보행 `5321/533` 85개, 자전거 `534` 13개 |
| traffic light | source geometry 152개 보존; association을 해소하지 못한 4개 warning |
| derived intersection | 144개 hull; 120 m 초과 broad hull 12개 warning |
| routing | source successor와 내부 segment의 native endpoint topology는 warning 허용; explicit graph 검사 pass |

유효한 OSM way를 만들 수 없어 생략한 boundary는 다음 5개다. 각 geometry는 서로 다른 XY
점이 하나뿐이며, validator는 이 목록을 non-degenerate coverage와 분리해 보고한다.

- `B2256W000461`
- `B2256W000950`
- `B2256W000951`
- `B2256W001367`
- `B2256W001595`

차로 또는 crossing association을 source 참조에서 해소하지 못한 signal은 다음 4개다.
geometry와 ID는 OSM에 남지만 regulatory element에는 묶지 않는다.

- `C1256W000070` (pedestrian)
- `C1256W000097` (pedestrian)
- `C1256W000121` (car)
- `LCS02` (car)

또한 6개 single-crosswalk record는 `link_id_list`에 명시적 blank 값을 갖는다. importer는
빈 참조를 관계로 만들지 않고 validator warning에 source ID와 위치를 남긴다.

## 파생 정책

- 원본 파일은 읽기 전용이며 출력은 기본적으로 별도 `data/derived`에 쓴다. CLI
  `--output-dir`를 지정하면 그 디렉터리에 쓴다.
- 접미사 duplicate는 strict recursive equality를 입증한 항목만 canonical ID로 접는다.
- source link는 좌·우 boundary fragment의 chainage event에서 1:N semantic segment로
  나뉜다. 각 relation은 source `mgeo:id`와 별도의 `mgeo:segment_id/index/count`, 시작·끝
  chainage를 함께 기록한다. 0.5 m 이내 접점 event는 source의 투영 jitter로 군집화하여
  zero-area sliver를 막고, endpoint 위치 보정은 기본 10 m 구간에 부드럽게 분산한다.
- source boundary way는 감사용 원본 feature copy다. lanelet member는 해당 segment에 맞춘
  clipped derived way이며, source boundary ID 목록, 의미 signature와 전체 clipped geometry가
  정확히 같고 endpoint identity를 안전하게 합칠 수 있을 때만 두 lanelet이 같은 way를 쓴다.
  일부 overlap 또는 단순 근접은 공유하지 않는다.
- 누락 lane bound는 중심선과 폭에서 만들고 `mgeo:synthetic=yes`로 표시한다.
- surface marking은 source ID당 하나의 multipolygon area를 만들고, 참조 source link마다
  centroid chainage를 포함하거나 가장 가까운 lanelet segment에 연결한다.
- `5321`과 `533`은 `subtype=crosswalk`, `participant:pedestrian=yes`이고 `533`만
  `mgeo:raised=yes`다. `534`는 `subtype=bicycle_crossing`,
  `participant:bicycle=yes`이며 pedestrian crossing으로 취급하지 않는다.
- `road_polygon_set=[]`이므로 intersection polygon은 junction road membership의 convex
  hull로 만든 derived 보조 geometry다. 120 m를 넘는 12개 broad hull을 포함하므로 정밀한
  drivable area, regulatory boundary 또는 routing 근거로 사용해서는 안 된다.
- controller phase는 동적 계획 데이터이므로 current signal state로 변환하지 않는다.
- 제공 전역경로 TXT는 이미 scene의 SIM local ENU다. 4,430개 점은 재투영하거나 단순화하지
  않고 HTML preview의 기본 활성 초록색 비교 layer에만 넣으며 Lanelet2 OSM에는 삽입하지
  않는다.
- 안전하게 endpoint를 공유하지 못한 successor와, exact bound를 공유하지 않는 lateral
  관계는 비표준 `KATRI_routing_graph.json`에 source 및 expanded segment graph로 보존한다.
  validator는 link/lanelet 존재, predecessor/successor, lane-change 필드와 relation ID 대응을
  검사하지만 graph의 모든 metadata나 consumer 적용 여부까지 보증하지 않는다. planner가
  이 fallback JSON을 명시적으로 읽어야 한다.
- 한국 교통규칙 plugin은 포함하지 않는다. 구조와 원본 속성 보존 검사는 특정 국가
  traffic-rule engine의 법적 판정이 아니다.

## Lanelet2 공식 validator 교차검사

ROS Noetic Lanelet2 1.2.2의 `lanelet2_validate`로 실제 OSM load 후
`mapping.bool_tags`, `mapping.mandatory_tags`, `mapping.duplicated_points`,
`routing.graph_is_valid`를 실행한 결과는 0 issue다. 한국 traffic-rule plugin이 없어 이
검사는 `de/vehicle` 규칙으로 graph를 생성한 구조 교차검사이며 한국 법규 판정은 아니다.

별도의 편집 품질 휴리스틱은 원본과 derived 감사 geometry가 가까이 공존하므로
`points_too_close` warning 23,282건을 낸다. topology를 만들면 안 되는 근접 endpoint를
단순 좌표 거리만으로 같은 Point ID로 합치지 않은 결과도 포함된다. endpoint taper 적용 후
`curvature_too_big`은 2,126건에서 502건으로 줄었지만, 원본/합성 경계의 급한 형상 warning은
남는다. 둘 다 parser/reference/routing 오류는 아니지만 curvature-sensitive consumer는 실제
사용 전 smoothing·resampling 정책을 별도로 검증해야 한다.
