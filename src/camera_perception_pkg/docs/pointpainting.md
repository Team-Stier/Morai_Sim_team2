# 전방 PIDNet-S PointPainting

학습 완료한 MORAI 6-class `best.pt`를 사용한다. 모델 구현은 공식
[XuJiacong/PIDNet](https://github.com/XuJiacong/PIDNet)의 commit
`4c158cf24ce432f0a8cb43364fae38d93cee0dc3`에서 `pidnet.py`, `model_utils.py`만
그대로 포함했다. 동봉한 MIT LICENSE를 유지한다. 모델 다운로드나 추가 학습은 없다.
RGB/ImageNet normalization, final semantic head, 원본 해상도 bilinear logits
보간(`align_corners=True`)과 class softmax를 사용한다. argmax는 발행하지 않는다.
학습 때의 보조 head도 유지해 479개 checkpoint tensor를 strict loading한다.

ROS는 Noetic/Python 3.8이며, PyTorch는 별도 Python 환경에 설치한다.
이번 PC에서 검증한 환경은 `/home/paik/venvs/morai-pidnet/bin/python`,
PyTorch `2.2.2+cu118`, NumPy `1.23.5`다. 해당 Python에 `rospkg`, `catkin_pkg`,
`PyYAML`, `netifaces`가 필요하고 ROS setup을 source해야 한다.
ROS 의존성은 package.xml에 선언한 rospy, sensor_msgs, cv_bridge를 사용한다.

```bash
source /opt/ros/noetic/setup.bash
source /home/paik/morai-artifacts/pointpainting-catkin/devel/setup.bash
roslaunch system_bringup_pkg pointpainting.launch \
  python:=/home/paik/venvs/morai-pidnet/bin/python \
  weights:=/home/paik/morai-artifacts/pidnet-training/output/Morai6/pidnet_s_morai6/best.pt
```

이 실행은 Camera inference와 PointPainting만 시작한다. 센서 수신은 기존
`morai_interface_pkg`가 제공한다. 입력은 승인된 전방 압축영상과 raw LiDAR뿐이다.
시스템 시작·조합은 `system_bringup_pkg`가 소유한다.
이번 라이브 검증에서는 별도 터미널에서 `roslaunch morai_interface_pkg cameras.launch`와
`roslaunch morai_interface_pkg lidar_bridge.launch enable:=true`를 사용했다.
후자는 중앙 UDP 계약의 `isolated_probe_activation_allowed: true` 범위의 격리 시험이다.
`runtime_activation_allowed: false`인 LiDAR를 새 system bringup에 추가하지 않는다.
MORAI 센서 destination은 해당 PC/Camera 9291·LiDAR 2368 설정과 일치해야 한다.
bag 재생은 `use_sim_time:=true`를 추가하고 `rosbag play --clock`을 사용한다.
같은 그래프에서 live 센서와 bag을 동시에 발행하지 않는다.

6-channel Image는 `cv_bridge.cv2_to_imgmsg(..., encoding='passthrough')`로
`32FC6`를 만든다. Noetic cv_bridge의 explicit encoding 검사는 4채널까지의
이름 표를 사용하므로 explicit `32FC6` 지정은 KeyError가 발생한다.
최신 pending 영상 하나만 유지하며 잘못된 JPEG와 inference 실패는 해당 프레임을 버린다.
카메라 입력 freshness는 World Model에서 두 source stamp를 기준으로 검사한다.

결과·제한사항은 [World Model 검증 문서](../../world_model_pkg/docs/pointpainting.md)를 참고한다.
