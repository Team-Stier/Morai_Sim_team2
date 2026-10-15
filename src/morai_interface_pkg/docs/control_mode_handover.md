# 실제 MORAI 제어 모드에 따른 송신

Q 키 토글 횟수와 MORAI 실제 상태가 어긋나고, 송신하는 `ctrl_mode=2`가
수동 선택을 덮어쓰는 문제를 수정한다. 일반 Ego Vehicle Status 네트워크는 사용하지 않는다.

- Producer: `morai_vehicle_status_bridge`, UDP 9099의 181-byte Competition Vehicle Status.
- Consumer: `morai_control_sender`; Controller/Safety의 공개 명령 경로는 유지한다.
- 내부 wire: `std_msgs/UInt64MultiArray`, `data=[ctrl_mode, ingress_monotonic_ns]`.
- 같은 호스트/부팅 세션에서만 사용한다. ingress에서 기록한 monotonic 시각을 callback에서 덮어쓰지 않는다.
- Auto(2)이고 ingress age가 0 이상 0.5초 미만일 때만 송신한다.
- Keyboard(1), 다른 모드, 초기 미수신, 잘못된 패킷, 통신 끊김이면 송신을 막는다.
- `control_mode_timeout_sec`는 런타임 YAML 설정이다. 수신 queue=1, non-latched.
- ROS time이 멈추거나 역행해도 mode watchdog은 monotonic 시각으로 동작한다.
- 수동 복귀 반응 시간에는 상태 전송 주기와 callback 지연이 포함된다.
  수동 상태를 수신하기 직전까지 이미 송신된 패킷을 취소할 수는 없다.
- 자동 제동이나 새 모드 선택 명령은 추가하지 않는다. 수동 제동은 운전자가 수행한다.

## 검증 (2026-10-16)

- 실제 로컬 MORAI의 181-byte UDP 수신과 `ctrl_mode=2`를 확인했다.
- catkin 3개 패키지 build: ros_architecture_pkg, common_msgs_pkg, morai_interface_pkg.
- morai_interface_pkg: 91 tests 통과. 실제 localhost UDP sink에서 초기/수동/미확인/timeout 차단과 Auto 송신,
  유효하지 않은 상류 명령 거부 및 ROS time 고정 상태의 monotonic watchdog을 확인한다.
- 내부 ROS 메시지 직렬화/역직렬화 후 producer → consumer 전달을 검증한다.
- launch XML/YAML 파싱과 중앙 interface diagram 생성 검사를 수행한다.
- TF의 host-specific saved sensor profile SHA 검사는 원래 workspace에서도 동일하게 실패한다.
  저장된 simulator profile이 중앙 TF evidence hash와 달라 발생하며 이번 수정에서 변경하지 않는다.
- 실제 수동↔자동 전환을 수행하는 MORAI closed-loop 검증은 운전자 조작과 함께 추가로 확인해야 한다.
