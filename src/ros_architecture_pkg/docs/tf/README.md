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

MORAI 공식 센서 문서는 장착 위치를 미터, 회전을 roll/pitch/yaw 도(deg) 단위로
설정한다고 설명한다. `candidate_ros_pose`의 회전은 라디안(rad)으로 변환한 후보값이다.
`base_link`와 MORAI sensor target pivot의 일치, 축 방향, 회전 부호와 순서는
MORAI 시뮬레이터에서 별도 검증해야 한다. 사용자 장착 위치 승인은 이 검증을
대신하지 않는다. 사용자 요청의 개발 범위에서 odom/base_link/gps_link/imu_link/lidar_link만
발행하며 Camera TF는 비활성을 유지한다. `physical_alignment_verified:false`와
`autonomous_driving_ready:false`를 보존한다.

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
| 3D LiDAR | `Lidar3D-6`, ID 6 | `2.000, 0.000, 1.500` | `0, 0, 0` | 0.10 s | 2026-09-11 사용자 정정 위치 |
| GPS | `GPS-4`, ID 4 | `0.000, 0.000, 1.300` | `0, 0, 0` | 0.20 s | 2026-09-10 사용자 승인 프로필 + 저장 파일 해시 |
| IMU | `IMU-5`, ID 5 | `0.000, 0.000, 0.000` | `0, 0, 0` | 0.02 s | 2026-09-10 사용자 승인 프로필 + 저장 파일 해시 |

Camera 근거 원본은 `참고파일들/2026_molit_comp_cam_set (1).json`이며 SHA-256은
`5c3da20597f44a57a1ecab83374bd652024126e6a09e33a800ddc89c222dcbd4`다.

2026-09-10 사용자가 지정한 IMU `[0, 0, 0] m`, GPS `[0, 0, 1.3] m`는
최신 저장 파일의 값과 일치한다. ID와 원본 frame 문자열도 파일에서 확인했다.
두 센서의 원본 RPY는 `[0, 0, 0] deg`다. 이 기록은 장착 위치에 대한 승인과
저장 파일 확인이며, 활성 loadout 전체나 ROS 축·pivot 정합의 검증 완료를 뜻하지 않는다.

```text
MoraiLauncher_Lin_Data/SaveFile/Sensor/25.S4.MolitComp03/
SensorInfo_2023_Hyundai_Ioniq5.json
```

| 증거 키 | SHA-256 | 범위 | 비고 |
|---|---|---|---|
| `launcher_saved_profile_user_confirmed` | `1f7432b56041d5e6c47ff44155c0d96e47893aaf978ab92879125eae31a3193f` | GPS, IMU | 2026-09-10 사용자 승인: IMU/GPS 위치 값 교체 |

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

현재 구현은 **frame 이름·부모 관계·후보 위치를 보존하는 계약**이다. 정적 TF가
실제로 발행되거나 축·pivot/부호 검증이 완료됐다는 의미가 아니다.

## LiDAR 장착 위치 갱신 (2026-09-11)

사용자 지정값 `base_link -> lidar_link` XYZ `[2.0, 0.0, 1.5] m`를 중앙 계약에 반영했다.
RPY는 `[0, 0, 0]`이며 기존 개발용 정적 TF 발행 승인을 유지한다.
정적 TF publisher는 중앙 YAML의 위치를 읽으므로 다음 실행부터 정정 좌표를 사용한다.
이미 실행 중인 publisher에는 재시작이 필요하며, 이번 좌표 정정은 실측 검증 완료를 뜻하지 않는다.
Producer `morai_interface_pkg`의 frame 및 consumer `lidar_perception_pkg`,
`localization_pkg`, `world_model_pkg`의 공개 I/O는 동일하며 장착 위치는 중앙 YAML을 참조한다.

검증: Noetic catkin 빌드, 정적 TF publisher 테스트 4개(좌표, 발행 게이트,
단일 parent와 cycle 검사 포함), LiDAR 중앙 계약 테스트와 인터페이스 다이어그램 검사를 통과했다.
중앙 계약 전체 테스트는 46개 중 45개 통과했으며, 외부 GPS/IMU 저장 프로필의
SHA-256이 기존 계약 기록과 달라 1개 실패했다. 변경 전 커밋에서도 같은 해시
불일치가 확인되며 이번 변경에서는 해당 기록을 수정하지 않았다.
실제 UDP 수신, 실행 중 TF 및 MORAI closed-loop는 이번 변경에서 검증하지 않았다.
