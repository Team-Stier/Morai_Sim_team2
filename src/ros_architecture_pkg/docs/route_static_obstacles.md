# 경로 주변 정적 장애물 계약 변경

사용자 요청에 따라 HdMap.static_obstacles를 추가한다. producer hd_map_server_node는
고정 MGeo object_set의 벽·가드레일·콘크리트 방호벽·가로등/표지판 기둥을 사용한다.
경로 15 m 이내이며 경로 노면과 높이 차 1.5 m 이내인 부분만 0.25 m 이하로 샘플링한다.
치수가 없는 원본은 XY 각 축 0.4 m를 보수적으로 확장한다. 이는 실측 크기가 아닌
개발 가정이다. 장애물 원본과 설정을 map_id에 포함한다. 물리 정합은 미검증이다.

Planner는 geometry-only 모드를 포함하여 Frenet, Hybrid A*, 전환/유지 경로의
충돌 검사에 센서 객체와 함께 사용한다. 원본 측정시각이나 WorldModel 객체는 바꾸지 않는다.
지도만으로 센서 상태/위치 상태의 주행 준비 조건을 우회하지 않는다.

기존 소비자 Localization, Route Manager, World Model은 추가 필드를 무시해도 되지만
HdMap MD5 변경 때문에 공유 메시지와 모든 producer/consumer를 함께 재빌드·재시작해야 한다.
기존 static_walls는 위치 정합용 원본 선으로 유지한다. 다른 공개 topic은 추가하지 않는다.

## 검증 결과 (2026-10-20)

고정 원본을 importer의 검증된 suffix clone 제거 규칙으로 읽으면 장애물은 53개다.
벽 3, 가드레일 6, 콘크리트 방호벽 3, 가로등 23, 표지판 기둥 18개다.
원본을 dedupe 없이 직접 읽는 조사에서는 방호벽 복제본 1개가 추가되므로 54개로 집계된다.
실제 load_static_map → HdMap 직렬화를 확인했다(전체 wire 약 9.02 MB, 정적 latched 발행).
정적 데이터는 측정시각 예측이나 stale 판정 없이 캐시를 사용한다.

대상 catkin 빌드, 지도 68개·Planner 146개·공유 메시지 44개 테스트와
인터페이스 다이어그램 일치 검사가 통과했다. 중앙 계약 46개 중 45개 통과,
기존 host별 MORAI saved profile SHA-256 불일치 검사 1개가 실패한다.
해당 extrinsic 계약과 테스트는 origin/main과 byte 단위로 같으며 이번에 변경하지 않았다.
launch XML·YAML·manifest·Python 구문도 검사했다.
실제 ROS callback/TF/UDP와 MORAI closed-loop 주행은 검증하지 않았다.

두께·지도 불확실성 확장은 실측값이 아니며 전봇대라는 별도 원본 분류는 없다.
오브젝트 높이는 노면과 같은 층의 원본 위치를 고르는 용도로만 사용한다.
랜덤 장애물은 여전히 센서와 World Model이 제공해야 한다.
