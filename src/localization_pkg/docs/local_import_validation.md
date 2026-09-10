# Localization 패키지 로컬 이식 검증

- 날짜: 2026-09-10
- 대상: 로컬 `PAIK`, push하지 않음
- 원본: `feature/localization_pkg`, `b1dfb34c9c60120b391e98c6b44802649dda13ce`
- 변경 범위: `src/localization_pkg/`만

EKF 코어, 후보 설정, 기존 ROS adapter와 설계를 가져왔다. 현재 중앙 계약에 없는
이전 메시지·node/topic/frame을 사용하는 adapter는 소스로만 보존하고 빌드·설치·launch에서
제외했다. 중앙 계약과 공통 메시지는 수정하지 않았다. 현재 I/O 표와 다이어그램은 유지했다.
기존 인터페이스 테스트는 현재 producer-consumer 경계 및 구형 adapter 실행 방지 검사로
갱신했다. Noetic gtest 링크를 위해 테스트 main을 추가했다.

검증 결과:

- Noetic catkin 전체 빌드 성공.
- EKF C++ 테스트 4개, 인터페이스 Python 테스트 2개 통과.
  catkin_test_results 집계는 중첩 gtest suite를 포함해 10 tests, 오류/실패/skip 0으로 출력한다.
- 중앙 계약 테스트 40개 중 39개 통과, 호스트 MORAI 프로필 부재 1개 skip.
- 다이어그램 중앙 계약 정합성, YAML/XML 파싱 통과.

이 결과는 EKF 코어와 소스 이식 검증이다. ROS callback, 승인 메시지 발행, map/odom 분리,
측정시각·reset·watchdog, GPS blackout 실주행과 MORAI closed-loop 검증을 뜻하지 않는다.
