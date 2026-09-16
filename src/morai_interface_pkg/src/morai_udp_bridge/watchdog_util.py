# -*- coding: utf-8 -*-
"""
watchdog_util.py
- 역할: LiDAR watchdog의 staleness 판정 등 ROS 무의존 순수 로직(단위 테스트용).
"""


# 함수이름: is_stale
# 기능: 기준 시각(reference_sec) 이후 timeout_sec을 초과했는지 판정한다.
#       reference_sec은 "마지막 수신 시각" 또는 (아직 미수신이면) "노드 시작 시각".
#       -> 시작 직후 grace 구간은 호출측이 reference=start_time으로 넘겨 자연 처리.
# 인자: now_sec, reference_sec, timeout_sec
# 반환값: 초과했으면 True(NO_POINTS), 아니면 False(EXTERNAL_OK)
def is_stale(now_sec, reference_sec, timeout_sec):
    return (now_sec - reference_sec) > timeout_sec
