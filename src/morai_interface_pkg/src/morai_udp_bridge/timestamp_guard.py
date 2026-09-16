# -*- coding: utf-8 -*-
"""ROS에 의존하지 않는 source packet timestamp 순서/미래시각 검증기."""


class PacketTimestampGuard(object):
    """source별 timestamp 역행·중복·미래값을 fail-closed로 거른다.

    미래 허용 오차는 아직 중앙 계약에서 승인되지 않았으므로 ingress 시각보다
    큰 source stamp는 엄격히 거부한다. 역행은 해당 프레임을 거부한 뒤 내부
    기준을 초기화하여 clock reset 이후 다음 정상 프레임부터 재동기화한다.
    """

    def __init__(self):
        self.last_accepted = None
        self.reset_count = 0

    @staticmethod
    def _valid_stamp(stamp):
        sec, nsec = stamp
        return sec > 0 and 0 <= nsec < 1000000000

    def check(self, packet_sec, packet_nsec, ingress_sec, ingress_nsec):
        packet = (int(packet_sec), int(packet_nsec))
        ingress = (int(ingress_sec), int(ingress_nsec))

        if not self._valid_stamp(packet):
            return "invalid"
        if not self._valid_stamp(ingress):
            return "invalid_ingress_clock"
        if packet > ingress:
            return "future"

        if self.last_accepted is not None:
            if packet == self.last_accepted:
                return "duplicate"
            if packet < self.last_accepted:
                self.last_accepted = None
                self.reset_count += 1
                return "regression_reset"

        self.last_accepted = packet
        return None
