# 경로별 YOLO 관측 구간

사용자 지정(2026-10-15): 기존 지도 번호 4번 우회전은 제외하고 1·2·3·5번은
정지선까지 경로상 남은 거리가 0~30m일 때만 Camera3 YOLO 후보를 받는다.
이는 대회 전체의 우회전 법규나 신호 면제를 정의하지 않는 코스 실행 설정이다.
[2026-10-16 규정 대조](rulebook_update_20261016.md)에서 공식 판정 교차로가
4개소임을 확인했지만, 이 정책의 1·2·3·5번과 공식 판정 대상의 일치가
검증됐다는 뜻은 아니다. 공식 그림·운영측 안내와 지도 ID 대응을 별도로 확인해야 한다.

원본은 `config/map/signal_observation_policy.yaml`이다. 정지선 ID와 신호등 ID는
정적 MGeo 연결에서 가져오며 station은 원본 전역경로 TXT와 정지선의 교차점이다.
해당 파일 hash와 station의 원본 정합은 테스트로 검사한다. 지도 미리보기는 같은
정책으로 4번을 제외하고 활성 구간을 주황색으로 표시한다.

공개 계약 변경: `/molit/route/context`에 `camera_perception_node` consumer를 추가한다.
producer와 message schema는 그대로다. Route Manager가 위치 추정으로 만든 진행도를
사용하며 Camera Perception에서 pose 추정이나 경로 매칭을 중복 구현하지 않는다.
해당 진행도는 정적 관측 구간 선택 용도이며 신호 색상/대상 선택/주행 결정은 아니다.
다른 producer/consumer의 입출력, TF, 차량 제어 출력은 변경하지 않는다.

Camera Perception은 중앙 정책을 launch의 `~signal_policy`로 로드한다.
기본 gate는 활성화되며 유효한 context가 없으면 후보를 만들지 않는다.
context의 map frame, 경로 hash, 유한한 비음수 진행도, 원본 stamp와 monotonic
수신 watchdog을 검사한다. 0.5초 만료·미래시각·시각 역행·다른 map·진행도 역행·
정지선 통과 시 이전 구간 결과를 사용하지 않는다. 차량 재배치와 다음 회차에서는
새 유효 context로 해당 구간에 다시 들어오면 재개한다.
카메라 측정 stamp를 유지하고 처리 완료 시각으로 바꾸지 않는다. 추론 후에도
구간과 freshness를 재검사해 구간 밖에서 끝난 GPU 결과를 폐기한다.

private `~signal_detections`의 `gate_active=false`와 빈 후보는 이전 결과 무효화다.
활성 출력의 `route_scope.signal_ids`는 관측할 수 있는 정적 head 집합이다.
private 후보는 `route_target_selected=false`, `driving_decision_valid=false`를 유지한다.
공개 HSV 관측의 후속 World Model 선택·Planner 연결은 traffic_signal_hsv.md를 따른다.
World Model이나 Planner가 private 진단 topic을 구독하는 연결은 추가하지 않는다.
단독 영상 진단에만 명시적 `signal_route_gate:=false`를 사용할 수 있다.

검증(2026-10-15): Camera 12개, HD Map 65개, 중앙 계약 46개, Bringup 24개
테스트 통과. Camera/HD Map catkin 빌드, 중앙 다이어그램 검사 통과.
원본 정지선 교차점 station, 30m 양쪽 경계, 정지선 통과, 4번 제외, stale/future/
clock reset, 추론 도중 이탈과 원본 카메라 stamp 보존을 검사했다.
실제 Camera3 UDP 입력 43프레임을 받은 3초 동안 RouteContext 없이 YOLO 영상 출력이
0회임을 확인했다. 차량으로 실제 네 구간을 통과하는 MORAI closed-loop 시험은 미수행이다.
