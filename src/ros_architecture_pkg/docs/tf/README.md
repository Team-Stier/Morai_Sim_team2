# TF 계약

이 문서는 [`frame_contract.yaml`](../../config/tf/frame_contract.yaml)과
[`sensor_extrinsics.yaml`](../../config/tf/sensor_extrinsics.yaml)의 판단 근거와
활성화 조건을 설명한다. 기계 판독 가능한 값의 원본은 두 YAML 파일이다.

## 승인된 frame tree

```text
map
└── odom
    └── base_link
        ├── camera_front_link
        │   └── camera_front_optical_frame
        ├── camera_left_link
        │   └── camera_left_optical_frame
        ├── camera_right_link
        │   └── camera_right_optical_frame
        ├── lidar_link
        ├── gps_link
        └── imu_link
```

- `map -> odom`: `localization_pkg`가 전역 보정을 반영해 동적으로 발행한다.
- `odom -> base_link`: `localization_pkg`가 연속적인 ego motion으로 동적으로 발행한다.
- `base_link -> sensor`: 센서 장착 위치를 나타내는 정적 변환이다.
- Camera `*_link -> *_optical_frame`: REP-103 optical convention을 나타내는 정적 변환이다.
- 정적 변환의 값은 `ros_architecture_pkg`가 승인하고, 전체 실행 시 단일 publisher는
  `system_bringup_pkg`가 조합한다.

`odom`은 연속 이동 추정이며 절대 위치 정답이 아니다. `map` 원점은 공식 HD Map의
좌표계와 local origin이 검증된 뒤 확정한다.

## 좌표 규약

차량과 정규화된 센서 frame은 ROS REP-103을 따른다.

```text
x: forward
y: left
z: up
```

Camera optical frame은 다음 축을 사용한다.

```text
x: right
y: down
z: forward
```

MORAI 공식 센서 문서는 장착 위치를 미터, 회전을 roll/pitch/yaw 도 단위로
설정한다고 설명한다. 하지만 Sensor Editor의 차량 기준 원점과 팀이 사용할
`base_link` 원점의 동일성, 회전 합성 순서와 부호는 아직 실시간으로 확인되지 않았다.
따라서 아래 값은 저장하되 현재 모든 정적 센서 TF는 `publish_enabled: false`다.

- [ROS REP-103](https://www.ros.org/reps/rep-0103.html)
- [ROS REP-105](https://www.ros.org/reps/rep-0105.html)
- [MORAI 센서 설정](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/sensors)
- [MORAI 센서 좌표계](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/sensor-coordinate-system)

## 센서 위치 근거

저장소의 Camera 원본과 로컬 MORAI 저장 프로필을 읽어 다음 값을 기록했다.

| 센서 | MORAI 식별자 | XYZ (m) | 원본 RPY (deg) | 설정 주기 | 현재 근거 |
|---|---|---|---|---:|---|
| Front Camera | `Camera-1`, ID 1 | `1.900, 0.000, 1.200` | `0, 2, 0` | 0.05 s | 저장소 원본과 로컬 프로필 일치 |
| Left Camera | `Camera-2`, ID 2 | `1.150, 0.650, 1.200` | `0, 10, 70` | 0.05 s | 저장소 원본과 로컬 프로필 일치 |
| Right Camera | `Camera-3`, ID 3 | `1.150, -0.650, 1.200` | `0, 10, 290` | 0.05 s | 저장소 원본과 로컬 프로필 일치 |
| 3D LiDAR | `Lidar3D-4`, ID 6 | `2.000, 0.000, 1.500` | `0, 0, 0` | 0.10 s | 로컬 저장 프로필만 확인 |
| GPS | `GPS-5`, ID 5 | `0.000, 0.000, 0.000` | `0, 0, 0` | 0.20 s | 로컬 저장 프로필만 확인 |
| IMU | `IMU-4`, ID 4 | `3.434, -0.354, 0.602` | `0, 0, 0` | 0.02 s | 로컬 저장 프로필만 확인 |

Camera 근거 원본은 `참고파일들/2026_molit_comp_cam_set (1).json`이며 SHA-256은
`5c3da20597f44a57a1ecab83374bd652024126e6a09e33a800ddc89c222dcbd4`다.

LiDAR/GPS/IMU 값은 다음 MORAI 저장 프로필에서 확인했지만 Simulator가 실행 중이지
않았으므로 현재 활성 loadout이라는 증거는 없다.

```text
MoraiLauncher_Lin_Data/SaveFile/Sensor/25.S4.MolitComp03/
SensorInfo_2023_Hyundai_Ioniq5.json
```

원본의 `Lidar3D-4` 문자열과 Sensor ID `6`은 숫자가 다르다. 이름의 숫자를 센서 ID로
재해석하지 않는다. MORAI 원본 frame 문자열은 evidence alias이고 중앙 ROS frame 이름은
아니다.

정적 sensor extrinsic은 시간에 따라 변하지 않는 calibration 관계다. Camera/LiDAR의
`header.stamp`나 `sensorPeriod`를 정적 TF의 시각으로 사용하지 않는다. 관측시각과 stale
판정은 별도의 [Timestamp 계약](../timestamp/README.md)이 담당한다.

## 정적 TF 활성화 게이트

다음 검증을 모두 통과하기 전에는 `/tf_static` publisher를 추가하지 않는다.

1. MORAI에서 실제 로드된 sensor profile 이름과 해시를 확인한다.
2. 비대칭 위치의 센서를 이용해 MORAI 위치축이 `x-forward, y-left, z-up`인지 확인한다.
3. Camera yaw와 pitch를 이용해 RPY 부호 및 회전 순서를 확인한다.
4. MORAI sensor target pivot과 `base_link` 원점의 위치를 확인한다.
5. Camera optical 축과 LiDAR raw point 축을 실제 수신 데이터로 확인한다.
6. YAML의 `verification_status`를 `runtime_verified`로 변경하고 명시적으로
   `publish_enabled: true`를 승인한다.
7. `system_bringup_pkg`에서 단 하나의 정적 TF publisher를 실행한다.
8. TF 단일 parent, cycle 부재와 실제 transform을 `tf2`로 검사한다.

현재 구현은 **frame 이름·부모 관계·후보 위치를 보존하는 계약**이다. 정적 TF가 실제로
발행되거나 센서 정합이 완료됐다는 의미가 아니다.
