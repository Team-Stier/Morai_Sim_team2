# Closedteam2 컨트롤 코어 이식

원본: https://github.com/Team-Stier/Closedteam2.git

고정 commit: `ff8eb424585183790165ef2f50042ec7c8517e4b` (2026-09-05).

`control_pkg/hybrid_path_tracking`의 Pure Pursuit, Stanley, path geometry,
HybridSupervisor 구현·헤더와 네 원본 단위시험을 수정 없이 보관한다.
`path_source_manager.hpp`는 supervisor의 PathSource 타입 의존성 때문에
포함한다. path manager 구현은 원본 supervisor 단위시험의 helper로만
컴파일하며 런타임 제어 라이브러리와 노드에 연결하지 않는다.

`control_pkg/longitudinal_control/src/longitudinal_control_node.cpp`의
`BoundedPiController` 클래스는 본문 그대로 `src/imported/bounded_pi.hpp`로
추출했다. include, namespace와 기존 Clamp helper만 독립 컴파일에 맞춰 붙였다.
기본 gain은 원본 기준이며 속도 입력 단위는 km/h다.

새 `controller.cpp`는 승인된 odom 경로에 두 조향기를 적용하고 원본 supervisor가
선택한 rad 값을 그대로 반환한다. `GLOBAL` enum은 원본의 두 제어기 혼합 모드를
선택하는 내부 표식이며 실제 입력은 전역경로가 아닌 Planner local trajectory다.
`vehicle_controller_node.cpp`는 네 공개 입력과 두 출력을 연결한다.

원본의 ROS package, custom messages, 원본 control adapter, speed policy,
mission/obstacle 판단과 UDP 출력은 가져오지 않았다. 계획과 Safety 책임은
현재 저장소의 패키지 경계를 유지한다. 추가 방어 계층은 사용자 지시로 제외했다.

이식한 snapshot에서 별도 LICENSE/COPYING 파일은 발견하지 못했다.
새 라이선스를 추정해서 부여하지 않았으며 원본 저장소와 commit을 기록한다.

빌드 연결 시 main에 이미 있던 TrackedObject/WorldModel `.msg`가
common_msgs_pkg의 메시지 생성 목록에서 누락된 점도 수정했다.
기존 World Model 소비자가 전체 workspace에서 빌드·import되도록 하는 등록 수정이다.
