# Localization 노드 구성도

이 문서는 `localization.launch`, `topics.yaml`, `ekf_local.yaml`,
`ekf_global.yaml`에 설정된 현재 실행 구조를 보여준다.

![Localization 노드 구성도](./localization_node_architecture.svg)

## 읽는 방법

- 실선 화살표: ROS topic 데이터 흐름
- 점선 화살표: `robot_localization/SetPose` service 호출
- `map -> odom`: Global EKF가 발행하는 TF
- `odom -> base_link`: Local EKF가 발행하는 TF

Local EKF의 `/localization/local/odometry`는 현재 패키지의 실행 노드가
구독하지 않는다. Local EKF의 주된 연결 역할은 `odom -> base_link` TF 발행이다.
Supervisor는 `/localization/global/odometry`만 검증하여
`/localization/odometry`로 relay한다.

현재 설정은 IMU와 vehicle twist가 이미 `base_link` frame이라고 가정한다.
센서 장착 위치를 나타내는 static TF와 Planner·Controller 연결은 이 저장소의
현재 Localization 구성에 없으므로 다이어그램에 포함하지 않았다.

Mermaid 원본은
[`localization_node_architecture.mmd`](./localization_node_architecture.mmd)에서
확인할 수 있다.


