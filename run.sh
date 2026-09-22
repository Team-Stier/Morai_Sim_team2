#!/usr/bin/env bash
# Workspace entry point for the complete MORAI development stack.
set -eo pipefail
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo '사용법: ./run.sh [--check] [rviz:=false] [send_to_morai:=false] [test_speed_cap_kph:=10.0]'
  echo '센서 → Localization/RViz → Frenet/Controller/UDP 실행. 종료: Ctrl+C'
  echo 'MORAI는 별도 실행하고 Cmd Control 127.0.0.1:9093을 Connect 상태로 둡니다.'
  exit 0
fi
check_only=false
if [[ ${1:-} == --check ]]; then
  check_only=true
  shift
fi
workspace=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
if [[ ! -f "$workspace/devel/setup.bash" ]]; then
  echo "빌드 환경이 없습니다: $workspace/devel/setup.bash (먼저 catkin_make 실행)" >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
cd "$workspace"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/morai-team2-run-${UID}.lock"
if ! flock -n 9; then
  echo '이미 run.sh가 실행 중입니다. 기존 터미널에서 Ctrl+C로 종료하세요.' >&2
  exit 1
fi
# Resolve names from the launch itself rather than defining another ROS contract.
expected_nodes=$(roslaunch --nodes system_bringup_pkg frenet_all.launch "$@")
/usr/bin/python3 - "$expected_nodes" <<'PY'
import socket
import sys
import rosgraph

socket.setdefaulttimeout(2.)
try:
    state = rosgraph.Master('/morai_run_preflight').getSystemState()
except (OSError, rosgraph.MasterError, rosgraph.MasterFailure):
    state = []  # roslaunch starts a master when none exists.
running = {node for group in state for _, nodes in group for node in nodes}
duplicates = sorted(running.intersection(sys.argv[1].split()))
if duplicates:
    sys.exit('이미 실행 중인 노드가 있습니다. 기존 실행을 종료하세요: ' + ', '.join(duplicates))
PY
if "$check_only"; then
  echo '실행 구성 확인 완료 (노드는 시작하지 않았습니다).'
  exit 0
fi
echo 'MORAI 전체 주행 스택 시작. 종료하려면 이 터미널에서 Ctrl+C를 누르세요.'
exec roslaunch system_bringup_pkg frenet_all.launch "$@"
