# 사전학습 PointPillars 연결

사용자 요청: 라벨링 데이터가 없으므로 사전학습 LiDAR 모델로 보행자와 차량을
먼저 구분한다. 추가 학습이나 정확도 평가를 완료했다는 의미가 아니다.

## 모델과 입력

- [OpenPCDet 공식 모델 목록](https://github.com/open-mmlab/OpenPCDet#nuscenes-3d-object-detection-baselines)의
  nuScenes `PointPillar-MultiHead`, `cbgs_pp_multihead.yaml`.
- 실행 소스: OpenPCDet `v0.5.2`, commit `b6fbf07fa0d6ca391037d02855a6e704f5478731`.
- 공식 가중치: [pp_multihead_nds5823_updated.pth](https://drive.google.com/file/d/1p-501mTWsq0G9RzroTWSXreIMyTUUpBM/view).
  SHA256 `0d241edcfc089a1d1901c2747ea9cb8cb3eec80c9e86a551727ac526e063e77d`.
  실행 전에 hash를 확인하며 모든 state tensor를 strict load한다.
- 입력: 승인된 raw PointCloud2의 FLOAT32 x/y/z/intensity. intensity는 원 값
  `[0,255]`를 유지하며 누락 시 임의 값으로 채우지 않는다. NaN 점은 제거한다.
- 다섯 번째 특징은 현재 스캔의 sweep age `0`이다. 과거 프레임·가짜 timestamp·
  시나리오 정답·객체 카탈로그를 모델 입력으로 사용하지 않는다.
- nuScenes 모델은 10 sweeps로 학습됐다. MORAI 단일 VLP16 스캔과 beam 수,
  지면 높이·센서 장착·반사강도 분포가 다르므로 miss/오분류 가능성이 크다.
  여러 스캔 정합·ego motion 보정·추적은 이번 구현에 포함하지 않는다.
- 모델 입력 범위는 사전학습 설정 `[-51.2,51.2] × [-51.2,51.2] × [-5,3] m`.
  `detector.yaml`의 ROI `x=[0,40], y=[-15,15], z=[-1.5,1] m`와 AABB가 겹치는
  결과만 발행한다. 박스를 ROI 경계에서 잘라 크기를 왜곡하지 않는다.
  private `~filtered_points`는 이 ROI 안의 raw XYZ이며 learned 모드에서는 voxel
  cluster를 뜻하지 않는다. DBSCAN 모드에서는 기존 VoxelGrid 결과다.

## 출력과 제한

중앙 [LiDAR 계약](../../ros_architecture_pkg/docs/lidar_detection_contract.md)이
공개 메시지 의미를 소유한다. 공개 node/topic/frame은 그대로다. 두 launch는
서로 대체 관계이며 동시에 실행하지 않는다.

| 원 모델 클래스 | 공개 분류 | RViz |
|---|---|---|
| pedestrian | PEDESTRIAN | 초록 + 클래스/점수 |
| car, truck, construction_vehicle, bus, trailer | VEHICLE | 파랑 + 클래스/점수 |
| barrier, motorcycle, bicycle, traffic_cone | OTHER | 주황 + 클래스/점수 |
| DBSCAN geometry | UNKNOWN | 기존 분홍 박스 |

`pedestrian`은 성별·성인 여부를 판별하지 않는다. `CargoBox`·벽 등 미학습
클래스의 범용 인식이나 DBSCAN fallback은 제공하지 않는다. 검출 0개를
주변이 비었다는 의미로 사용하면 안 된다.

기본 모델 점수 임계값은 0.3이며 `config/learned.yaml`에서 조정한다. 점수는
보정된 확률이 아니며 `confidence=-1`을 유지한다. 모델이 예측한 oriented box를
둘러싸는 AABB를 발행하므로 회전된 차량의 size는 실제 차체 치수보다 커질 수
있다. point_count는 그 oriented box 안의 유한한 raw 점 개수다. 지지점 0개
박스는 발행하지 않는다. 모델의 속도 출력은 사용하지 않는다.

한 worker가 추론하며 대기 슬롯은 최신 scan 1개로 제한한다. 원 측정 stamp를
보존하고 frame 재지정이나 TF 발행은 하지 않는다. 단절·clock reset·모델
로드 실패·추론 실패·0.8초 개발 실행 예산 초과를 FAULT로 보고한다. 이 예산은
실측된 주행 freshness 한계가 아니다. `calibration_verified=false`,
`freshness_verified=false`, `ready=false`, `stop_required=true`를 유지한다.
모델 로드에 실패해도 status heartbeat는 계속 발행한다.

## 설치 재현 (Ubuntu 20.04 / Python 3.8 / NVIDIA)

드라이버·ROS 시스템 환경을 바꾸지 않고 사용자 디렉터리에 설치한다.
아래 명령은 저장소 root에서 실행한다. 가중치와 CUDA/모델 환경은 Git에 넣지 않는다.

```bash
python3 -m venv --system-site-packages "$HOME/.local/share/morai-lidar-model-env"
source "$HOME/.local/share/morai-lidar-model-env/bin/activate"
python -m pip install --upgrade pip==24.3.1 setuptools==69.5.1 wheel==0.45.1
python -m pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu118
python -m pip install -r src/lidar_perception_pkg/config/pretrained-requirements.txt
git clone --branch v0.5.2 --depth 1 https://github.com/open-mmlab/OpenPCDet.git "$HOME/.local/share/morai-OpenPCDet"
git -C "$HOME/.local/share/morai-OpenPCDet" apply "$PWD/src/lidar_perception_pkg/docs/patches/openpcdet-pytorch2.patch"
```

CUDA extension 빌드에는 runtime wheel 외에 CUDA 11.8 개발 헤더와 nvcc가
필요하다. [NVIDIA 공식 재배포 manifest](https://developer.download.nvidia.com/compute/cuda/redist/redistrib_11.8.0.json)의
linux-x86_64 `cuda_nvcc`, `cuda_cudart`, `cuda_cccl` archive를 각 SHA256 확인 후
하나의 `$HOME/.local/share/morai-cuda-11.8` 아래에 합친다. archive의 최상위
디렉터리 한 단계만 제거하며 `bin/include/lib/nvvm` 구조를 보존한다.
각 패키지의 읽기 전용 LICENSE는 다른 이름으로 보존하여 덮어쓰기 충돌을 피한다.

```bash
export CUDA_HOME="$HOME/.local/share/morai-cuda-11.8"
export TORCH_CUDA_ARCH_LIST=8.6  # RTX 3060. 다른 GPU는 해당 compute capability 사용.
export MAX_JOBS=3
export CPATH="$VIRTUAL_ENV/lib/python3.8/site-packages/nvidia/cusparse/include:$VIRTUAL_ENV/lib/python3.8/site-packages/nvidia/cublas/include:$VIRTUAL_ENV/lib/python3.8/site-packages/nvidia/cusolver/include:$VIRTUAL_ENV/lib/python3.8/site-packages/nvidia/curand/include"
cd "$HOME/.local/share/morai-OpenPCDet"
python setup.py build_ext --inplace
```

호환성 patch는 PointNet2 코드의 미사용 `THC/THC.h`와 `THCState` 선언을
제거하고, BEV backbone의 폐기된 NumPy `np.int` 별칭을 동일한 builtin `int`로
바꾼다. THC 삭제는 최신 OpenPCDet에도 반영돼 있다. PointPillars 가중치·
네트워크·NMS·학습 파라미터는 변경하지 않는다. 기존 다른 데이터셋 의존성
추가를 피하기 위해 재현 소스를 v0.5.2로 고정했다.
또한 OpenPCDet의 버전 조회가 ROS 실행 디렉터리 대신 실제 모델 저장소에서
git SHA를 읽도록 작업 디렉터리를 명시한다.

공식 가중치를 모델 저장소의 `pp_multihead_nds5823_updated.pth`에 저장한 뒤
위 SHA256과 일치하는지 확인한다. 임의 출처의 pickle 가중치를 로드하지 않는다.

## 실행과 검증

이 변경을 빌드한 worktree에서 실행해야 한다. ROS1 메시지 MD5가 바뀌므로
이전 검출 노드와 이전 메시지를 로드한 시각화 프로세스를 함께 재시작한다.
MORAI UDP bridge/watchdog은 동일한 기존 raw 토픽을 계속 제공한다.

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4 -DCATKIN_WHITELIST_PACKAGES='common_msgs_pkg;ros_architecture_pkg;lidar_perception_pkg;visualization_pkg;hd_map_pkg'
source devel/setup.bash
roslaunch lidar_perception_pkg learned_lidar.launch
# 별도 터미널, 같은 devel을 source한 뒤:
roslaunch visualization_pkg lidar_debug.launch
```

모델 경로·Python·점수 기준은 launch의 `model_repository`, `checkpoint`, `python`,
`score_threshold` 인자로
바꿀 수 있다. 기존 DBSCAN은 `lidar_perception_pkg.launch`로 선택한다.

테스트는 row padding·잘못된 XYZI·회전 박스·지지점·분류 계약·원 stamp 보존·
추론 실패·실행 예산·추론 중 reset·모델 로드 실패·RViz 색상/라벨을 확인한다.
GPU 없는 테스트는 가짜 backend를 주입하며 정확도를 증명하지 않는다.
실제 가중치 로드와 센서 추론의 관찰 결과는 별도 runtime 기록으로 남긴다.

[2026-09-14 로컬 실행 결과](pretrained_runtime.md): 차량 후보와 RViz 연결은
확인했으나 보행자 검출은 확인하지 못했다. 성능 수치와 검증 한계를 함께 본다.
