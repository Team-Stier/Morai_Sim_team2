# MORAI 센서 UDP 연결 실행 절차

## 범위

이 절차는 기존에 구현된 Camera 3대, GPS, IMU, VLP16 LiDAR를 MORAI UDP에서
받아 ROS1 topic으로 확인하는 **격리된 연결 시험**이다. Localization이나 자율주행
알고리즘은 실행하지 않는다. Competition Vehicle Status, CollisionData와
Ego Ctrl Cmd도 포함하지 않는다.

## 1. ROS 의존성

ROS Noetic workspace에서 다음 패키지가 필요하다.

```bash
sudo apt install ros-noetic-velodyne-driver \
  ros-noetic-velodyne-pointcloud ros-noetic-velodyne-msgs ros-noetic-nodelet
catkin_make
source devel/setup.bash
```

## 2. MORAI Network Settings

MORAI의 각 센서 destination IP는 ROS를 실행하는 PC의 실제 IPv4 주소로 지정한다.
기본 수신 포트는 다음과 같다. 포트는 대회 고정값이 아니라 Team2 개발 설정이다.

| 센서 | 수신 포트 | ROS topic |
|---|---:|---|
| Camera Front | 9291 | `/molit/sensors/camera/front/image/compressed` |
| Camera Left | 9293 | `/molit/sensors/camera/left/image/compressed` |
| Camera Right | 9295 | `/molit/sensors/camera/right/image/compressed` |
| GPS | 7801 | `/molit/sensors/gps/fix` |
| IMU | 7802 | `/molit/sensors/imu/data` |
| VLP16 LiDAR | 2368 | `/molit/sensors/lidar/points` |

규정 범위는 GPS 30 Hz 이하, IMU 50 Hz 이하, VLP16 15 Hz 이하(권장 10 Hz 이하),
Camera 30 Hz 이하다. 고정 Camera 3대의 위치·각도·FOV는 변경하지 않는다.

## 3. 실행

Camera/GPS만 실행하면 다음 명령으로 충분하다.

```bash
roslaunch morai_interface_pkg morai_interface_pkg.launch
```

기존 센서 브리지 5종을 모두 연결 시험하려면 IMU와 LiDAR를 명시적으로 켠다.

```bash
roslaunch morai_interface_pkg morai_interface_pkg.launch \
  start_imu:=true start_lidar:=true start_lidar_watchdog:=true
```

## 4. 수신 확인

다른 터미널에서 같은 workspace를 source한 뒤 확인한다.

```bash
rostopic list | sort
rostopic hz /molit/sensors/camera/front/image/compressed
rostopic hz /molit/sensors/camera/left/image/compressed
rostopic hz /molit/sensors/camera/right/image/compressed
rostopic hz /molit/sensors/gps/fix
rostopic hz /molit/sensors/imu/data
rostopic hz /molit/sensors/lidar/points
rostopic echo -n 1 /molit/sensors/lidar/status
```

성공 조건은 여섯 데이터 topic이 실제 메시지를 계속 받고 LiDAR status가 `True`가
되는 것이다. 단순히 node가 떠 있거나 topic 이름이 보이는 것만으로 성공으로
판정하지 않는다. GPS blackout 구간에서는 GPS fix 유효성 상실이 정상일 수 있다.

## 5. 패킷이 보이지 않을 때

ROS보다 먼저 UDP가 PC에 도착하는지 확인한다.

```bash
sudo tcpdump -ni any 'udp port 9291 or udp port 9293 or udp port 9295 or udp port 7801 or udp port 7802 or udp port 2368'
```

- tcpdump에도 없으면 MORAI destination IP/port, 실행 PC NIC와 방화벽을 확인한다.
- UDP는 보이지만 ROS topic이 없으면 launch 로그의 packet 길이·파싱 실패를 확인한다.
- IMU는 정지·직진·좌회전·우회전 rosbag으로 축과 단위를 확인하기 전에는
  Localization의 승인 입력으로 사용하지 않는다.
- LiDAR는 packet 수신 후 PointCloud2의 `frame_id=lidar_link`, 회전율과 좌표축을
  확인한다. 정적 TF는 별도 검증 전까지 발행하지 않는다.

## Vehicle Status 주의

MORAI 공식
[`EgoVehicleStatus.py`](https://github.com/MORAI-Autonomous/MORAI-NetworkModule/blob/24.R2.0/EgoNetwork/Publisher/EgoVehicleStatus.py)는
일반 EgoVehicleStatus 수신 예시다. 연결된 구조에는 대회 Competition Vehicle
Status에서 제공되지 않는 위치, 횡속도, 가속도와 타이어 정보가 포함된다. 따라서
현재의 legacy Vehicle Status launch는 이 센서 시험에 포함하지 않는다.
