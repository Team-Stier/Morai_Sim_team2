# Planning visualization observer

## 목적과 경계

이 observer는 전역경로, 지역경로, HD map과 함께 실제 localization pose의
이동을 RViz에서 확인하기 위한 read-only 구성요소다. ROS 공개 이름과 의미는
`ros_architecture_pkg/config/interface_contract.yaml`만이 정의한다. 이 노드는
actuator command를 발행하지 않고 planning 결과를 수정하지 않는다.

## 입력 승인 조건

한 odometry sample은 다음 조건을 모두 만족할 때만 accepted trace에 추가된다.

1. `header.frame_id == map`이고 `child_frame_id == base_link`이다.
2. timestamp가 양수이고 현재 ROS 시각보다 미래가 아니며 age가 0.20 s 이하이다.
3. pose, twist, pose covariance와 twist covariance의 모든 값이 finite다.
4. quaternion norm과 1.0의 차이가 0.001 이하이다.
5. 이전 accepted timestamp보다 엄격히 크다.

잘못된 sample을 보정하거나 최근 정상 pose로 대체하지 않는다. 거부 이후 vehicle
marker는 삭제되고, 과거 trace는 진단을 위해 남지만 더 이상 늘어나지 않는다.
마지막 accepted sample의 age가 0.25 s를 넘는 경우에도 marker를 `DELETE`한다.
`ADD` marker의 lifetime은 accepted sample timestamp + 0.25 s까지의 남은
시간이다. 만료된 sample은 `ADD`하지 않으며, 같은 sample을 반복
발행해도 만료 시각이 늘어나지 않아 node 종료나 통신 단절 시 stale
자세가 RViz에 남는 것을 막는다.

## 차량 envelope

`base_link`는 rear-wheel midpoint다. 시각화 cube는 대회 차량인 2023 Hyundai
IONIQ 5의 길이 4.635 m, 폭 1.892 m, 높이 2.434 m를 사용한다. rear overhang
0.790 m를 적용하면 cube 중심은 rear axle보다 차량 x축 전방으로
`4.635 / 2 - 0.790 = 1.5275 m` 떨어진다. 이 local offset과 높이 절반 offset은
입력 quaternion으로 회전한 후 map pose에 더한다.

cube는 규정 접촉 판정기나 planner collision model이 아니다. 실제 wheel contact
검증은 map boundary, tire footprint, localization 오차와 controller tracking 오차를
포함한 별도 safety 검증이 필요하다.

## 단위 검증

ROS가 없는 개발 환경에서도 다음 순수 로직을 검증할 수 있다.

```bash
PYTHONPATH=src/runtime_evaluation_pkg/src \
python3 -m unittest discover -s src/runtime_evaluation_pkg/test -p 'test_*.py'
```

검증 대상은 frame/time/finite/quaternion reject, rear-axle body-center 변환,
bounded trace와 out-of-order timestamp 거부다. ROS publisher/subscriber, `/clock`,
RViz 렌더링은 ROS1 Noetic 환경에서 별도로 통합 검증해야 한다.
