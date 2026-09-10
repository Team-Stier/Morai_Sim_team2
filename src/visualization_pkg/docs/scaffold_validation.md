# 패키지 골격 검증 (2026-09-10)

현재 단계는 단일 RViz 화면을 위한 패키지 파일 생성이다.
실행 node, 구독·발행 topic, RViz display 설정은 아직 없다.
전체 실행 조합은 system_bringup_pkg가 소유한다.

- ROS Noetic catkin 빌드 성공 (15개 패키지 구성).
- package.xml, launch XML 및 중앙 YAML 파싱 성공.
- 중앙 계약과 공개 I/O 그림 정합성 검사 통과.
- 중앙 계약 테스트 40개 중 39개 통과, 호스트 MORAI 프로필 부재 1개 skip.
- RViz 화면 표시, 실제 topic 및 TF 연결은 구현·검증 전이다.

로컬 PAIK에서만 작업하며 push하지 않는다.
