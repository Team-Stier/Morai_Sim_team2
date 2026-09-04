# -*- coding: utf-8 -*-
"""
udp_sender.py
- 역할: 목적지 IP/포트로 UDP datagram을 보내는 얇은 래퍼(수신용 UdpReceiver의 대칭).
        ROS에 의존하지 않으므로 loopback 테스트에 그대로 쓸 수 있다.
- 주요 클래스: UdpSender
"""

import socket


class UdpSender(object):
    """지정한 목적지로 datagram을 전송한다."""

    # 함수이름: __init__
    # 기능: UDP 소켓을 만들고 목적지를 보관한다.
    # 인자: destination_ip, destination_port
    def __init__(self, destination_ip, destination_port):
        self.destination = (destination_ip, int(destination_port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._closed = False

    # 함수이름: send
    # 기능: bytes 한 개를 목적지로 전송한다.
    # 반환값: 보낸 바이트 수
    def send(self, data):
        if self._closed:
            return 0
        return self.socket.sendto(data, self.destination)

    # 함수이름: close
    # 기능: 소켓을 닫는다(중복 호출 안전).
    def close(self):
        if not self._closed:
            self._closed = True
            try:
                self.socket.close()
            except OSError:
                pass

    def __del__(self):
        self.close()

