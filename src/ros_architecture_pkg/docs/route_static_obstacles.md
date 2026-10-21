# 경로 주변 정적 장애물 계약 변경

사용자 요청에 따라 HdMap.static_obstacles를 추가한다. producer hd_map_server_node는
고정 MGeo object_set의 벽·가드레일·콘크리트 방호벽·가로등/표지판 기둥을 사용한다.
경로 15 m 이내이며 경로 노면과 높이 차 1.5 m 이내인 부분만 0.25 m 이하로 샘플링한다.
치수가 없는 원본은 XY 각 축 0.4 m를 보수적으로 확장한다. 이는 실측 크기가 아닌
개발 가정이다. 장애물 원본과 설정을 map_id에 포함한다. 물리 정합은 미검증이다.

이번 병합은 지도 장애물의 생성·발행과 공유 메시지·중앙 계약만 포함한다.
Planner의 Frenet/Hybrid A* 및 유지 경로 충돌 검사 연결은 포함하지 않는다.
기존 소비자는 추가 필드를 무시하며, 장애물 회피 동작은 이 변경으로 추가되지 않는다.

기존 소비자 Localization, Route Manager, World Model은 추가 필드를 무시해도 되지만
HdMap MD5 변경 때문에 공유 메시지와 모든 producer/consumer를 함께 재빌드·재시작해야 한다.
기존 static_walls는 위치 정합용 원본 선으로 유지한다. 다른 공개 topic은 추가하지 않는다.

## 원본 전체 변경의 검증 기록 (2026-10-20)

아래는 Planner 연결을 포함한 원본 브랜치의 기록이며, 지도만 병합한 결과의 검증과 구분한다.

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

## 지도와 Q 수정 병합 검증 (2026-10-21)

사용자 요청으로 장애물 브랜치의 지도 생성·발행 부분만 반영했다.
path_planning_pkg는 origin/main과 동일하며 장애물 충돌 검사 연결은 제외했다.
Q 전환 수정 두 커밋은 함께 반영했다.

- 전체 catkin_make 빌드 통과 (ROS Noetic, /usr/bin/python3).
- 지도 68개, 공유 메시지 44개, MORAI 통신 94개, bringup 25개, 기존 Planner 146개 테스트 통과.
- 실제 고정 지도에서 장애물 53개 생성 및 hd_map_server_node의 HdMap 변환·직렬화 왕복 확인 (9,024,655 bytes).
- 중앙 계약 46개 중 45개 통과. 로컬 saved sensor profile SHA 불일치 1건은 기존 작업 폴더에서도 동일하게 재현됐으며 관련 TF 계약과 테스트는 main과 동일하다.
- 인터페이스 다이어그램 일치 및 변경 YAML/XML/Python 구문 검사 통과.
- 실제 MORAI closed-loop, ROS topic callback 및 TF/UDP 실주행은 이번 병합에서 재검증하지 않았다.
- HdMap MD5가 바뀌므로 실행 중인 producer/consumer 전체를 재빌드·재시작해야 한다.
