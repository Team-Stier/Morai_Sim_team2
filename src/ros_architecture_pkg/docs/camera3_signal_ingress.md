# Camera3 신호등 YOLO 입력 연결

사용자가 만든 MORAI Camera-3는 제공 센서셋의 고정 Right Camera와 다른
신호등용 개발 카메라다. 기존 right 입력을 재사용하지 않고 아래 경계를 추가한다.

| 항목 | 값 |
|---|---|
| UDP 소유자 / node | morai_interface_pkg / morai_camera_signal |
| 수신 포트 | 9297 (MORAI host 9296) |
| 공개 입력 | /molit/sensors/camera/signal/image/compressed |
| 타입 / consumer | sensor_msgs/CompressedImage / camera_perception_node |
| frame | camera_signal_optical_frame |
| timestamp | morai_camera_packet_frame_time, 파생 영상은 원본 header 보존 |
| rate / queue | 최대 30 Hz / publisher 2, inference 최신 1장 |
| timeout / invalid | 청크 재조립 0.5초, 잘못된 JPEG·timestamp 폐기 |

Camera3 위치 [1.8,0,1.3]m, 자세 [0,340,0]deg와 FOV 90도를 보존한다.
학습 RGB와 같은 1920×1440 해상도를 사용하며 Bounding Box 송출은 끈다.
camera_signal의 mount/optical TF는 미검증이므로 발행하지 않는다.
카메라 추가는 기존 front/left/right frame 의미를 변경하지 않는다.

최초 입력 연결은 개발용 Camera3 수신과 YOLO 신호등 하우징 후보 영상까지였다.
원본 측정시각을 유지하는 private `~signal_image`와 `~signal_detections`는
진단용이며 경로 대상 ID, 신호 상태, 주행 가능 여부를 승인하지 않는다.
후속 [HSV 신호 연결](traffic_signal_hsv.md)에서 공개 CameraSignalArray와
WorldModel.traffic_signals를 추가하고 World Model의 지도 대상 선택, Planner의
정지·재출발 및 RViz 표시를 구현했다. 다른 차선 및 보행자 신호를 구분하는
지도 대응과 시간 정렬은 World Model이 소유한다. 본선 고정 3카메라의 ID와
자유 카메라 배치는 별도 확인한다.
