# 실행 모델 가중치

현재 실행에 필요한 PIDNet·YOLO 가중치를 Git에 포함한다. 별도의 학습 결과
디렉터리 없이 저장소의 파일을 읽으며 원본 학습 파일은 보존한다.

| 파일 | 용도 | SHA256 |
|---|---|---|
| `pidnet_s_morai6.pt` | 기본 PIDNet-S 6클래스 추론 | `dcdc0de7e1f1ea9b425157330ffcbf46c47c6f2da85f05cd7165b6fb2a4ddf95` |
| `camera3_route_signal.pt` | 기본 Camera3 YOLO11n 신호등 하우징 검출 | `e2a5e1569913fbb7a16db4efc3e958875ab2abf9c862575f6e753ea40b223b17` |
| `yolo26n.pt` | 보관용 YOLO 기본 가중치, 현재 launch에서는 사용하지 않음 | `9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef` |

PIDNet은 기존 `morai-artifacts/pidnet-training/output/Morai6/pidnet_s_morai6/best.pt`를
내용 변경 없이 복사했다. 학습한 YOLO의 출처는
[Camera3 문서](../src/camera_perception_pkg/docs/camera3_yolo.md)를 참고한다.
`run.sh`, `competition.launch`, `frenet_rddf.launch`, `pointpainting.launch`와
카메라 패키지 단독 launch는 저장소 위치에서 PIDNet 가중치를 찾는다.
`pidnet_weights`/`signal_weights` 인자(단독 launch는 `weights`)로 교체할 수 있다.

다른 PC에서는 Ubuntu 20.04·ROS1 Noetic, CUDA 지원 GPU/드라이버와 Python 의존성을
먼저 준비하고 저장소를 빌드해야 한다. 가상환경·ROS·드라이버는 Git에 포함하지 않는다.
기본 통합 실행은 `~/venvs/morai-yolo/bin/python`에서 두 모델을 함께 실행한다.
저장소 루트에서 Python 3.8 가상환경을 준비하는 예시는 다음과 같다.

```bash
python3 -m venv --system-site-packages "$HOME/venvs/morai-yolo"
"$HOME/venvs/morai-yolo/bin/python" -m pip install -r src/camera_perception_pkg/config/signal_requirements.txt
git submodule update --init --recursive
source /opt/ros/noetic/setup.bash
# ROS 의존성 설치는 루트 README의 rosdep 절차를 따른다.
catkin_make
./run.sh --check
./run.sh
```

아래 requirements는 현재 두 모델을 함께 실행하는 환경에서 확인한 버전을 기록한다.
새 PC에 이 절차로 환경을 새로 설치하는 검증은 별도로 필요하다.
YOLO를 끄면 기본 PIDNet 전용 환경 `~/venvs/morai-pidnet/bin/python`을 사용하므로,
동일 환경을 재사용하려면 다음처럼 지정한다.

```bash
./run.sh camera3_yolo:=false pidnet_python:="$HOME/venvs/morai-yolo/bin/python"
```

MORAI 실행·센서 설정·네트워크 연결은
[Bringup 안내](../src/system_bringup_pkg/README.md)를 따른다.
