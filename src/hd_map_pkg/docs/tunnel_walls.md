# 터널 벽 원본 복원 (2026-09-23)

- 원본: 고정된 MORAI KATRI MGeo `object_set.json`의 `wall` 객체
  `C3256W000003`(18점), `C3256W000005`(22점).
- map 변환: 원본 local XYZ + [2795, -1300, 0]. 중앙 EPSG:32652 원점 사용.
- 차량의 현재 추정 위치, 전역경로 또는 체크포인트를 벽 배치 관측으로 사용하지 않는다.
- 모든 원본 꼭짓점과 ID를 보존한다. 물리 정합은 미검증이다.
- 원본 높이는 빈 배열이다. JSON 산출물은 높이를 null로 기록한다.
  RViz의 5 m 면 높이는 시각화 파라미터이며 LiDAR 정합 데이터가 아니다.
- HTML은 원본 벽선을 표시하며, RViz는 원본 z에서 76개 삼각형으로 면을 만든다.
- 기존 HdMap wire는 유지하고 `/molit/map/static_walls`의 StaticWallMap을 추가했다.
  Localization이 이 메시지와 raw LiDAR를 받아 측정시각에 벽 법선 방향만 보정한다.
  진행방향 퇴화는 보정할 수 없으며, 물리 정합은 미검증이다.

검증: HD Map 단위 테스트 56개, 지도 마커 테스트 5개, Visualization 계약 테스트
2개 통과. 두 패키지 catkin 빌드, 중앙 인터페이스 다이어그램 검사, launch XML 및
변경 YAML 로딩 통과. 원본 데이터 → map 좌표 → RViz 마커에서 벽 2개,
40개 꼭짓점과 228개 삼각형 꼭짓점을 확인했다.

현장 원본 센서 기록은 저장소 외부
`/home/paik/morai-artifacts/tunnel-wall-survey-20260923/raw.bag`에 보존했다.
이 기록만으로 차량의 절대 위치 또는 벽의 물리 정합을 확정하지 않았다.
