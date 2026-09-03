# hd_map_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- 공식 지도 원본과 파생 HD Map의 버전·해시·좌표 메타데이터 관리
- 차선 중심선/경계, 실선/중앙선, topology와 drivable area 표현
- 정지선·교차로·신호등 연결, 속도 규칙과 Localization landmark 표현
- 정적 지도 로더, 변환기와 geometry/topology 검증
- 공식 전역경로·체크포인트·Link ID와 지도 레이어의 대응 근거 제공

## 담당하지 않는 범위

- 실시간 ego pose, 동적 객체, route 진행 상태
- 장애물 회피 local trajectory와 제어
- sample scene의 NPC·신호 상태를 정적 지도 정답으로 저장

## 현재 자료의 한계

`참고파일들/2026_molit_comp_global_path (3).txt`는 4,430개 XYZ 포인트의 전역경로이지 HD Map이 아니다. 차선 경계, 연결 관계, 정지선, 신호 연계와 주행 가능 영역 원본이 아직 없다. 필요한 레이어와 실제 시뮬레이터 정합을 검증하기 전에는 HD Map을 완료로 표시하지 않는다.

공식 맵명 `R-KR_PG_K-City_2025`와 sample scene의 `R_KR_PR_K-city_2025`도 다르므로 임의로 동일한 맵으로 확정하거나 원본을 수정하지 않는다.

## 논리 입출력

- 입력: 사용 권한이 확인된 공식 지도 자료와 immutable 원본
- 출력: 버전·해시·좌표계가 명시된 정적 map layers와 검증 상태

## 완료 기준

- 중앙 좌표계와 원점 검증
- 모든 필수 레이어의 schema·hash·coverage 기록
- 전역경로·체크포인트와 topology 대응 검증
- 시뮬레이터 화면 및 Localization landmark 정합 근거
- 원본과 파생물의 재현 가능한 변환 절차

## 디렉터리

- `config/`: 로더, layer와 validation 설정
- `docs/`: 원본 출처, schema, 변환·정합 보고서
- `launch/`: HD Map 단독 로드/검사
- `src/`: map parser, converter와 validator 구현
