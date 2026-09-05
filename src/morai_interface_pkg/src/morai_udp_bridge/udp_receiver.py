# -*- coding: utf-8 -*-
"""
udp_receiver.py
- 역할: UDP 소켓을 bind하고 timeout 기반으로 datagram을 수신하는 얇은 래퍼.
        ROS에 의존하지 않으므로 loopback 통합 테스트에 그대로 쓸 수 있다.
- 주요 클래스: UdpReceiver
"""

import socket


class UdpReceiver(object):
    """지정한 ip/port에 bind하고 datagram을 하나씩 반환한다."""

    # 함수이름: __init__
    # 기능: UDP 소켓을 만들고 bind하며 수신 timeout을 설정한다.
    # 인자: bind_ip, port, buffer_bytes(수신 버퍼 크기), timeout_sec(recv 대기)
    def __init__(self, bind_ip, port, buffer_bytes=2048, timeout_sec=0.5):
        self.bind_ip = bind_ip
        self.port = int(port)
        self.buffer_bytes = int(buffer_bytes)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 ** 20)
        self.socket.bind((self.bind_ip, self.port))
        self.socket.settimeout(timeout_sec)
        self._closed = False

    # 함수이름: receive
    # 기능: datagram 하나를 수신한다. timeout이면 None을 반환해
    #       호출자가 종료 여부를 확인할 수 있게 한다.
    # 반환값: (bytes, (ip, port)) 또는 timeout 시 None
    def receive(self):
        if self._closed:
            return None
        try:
            data, sender = self.socket.recvfrom(self.buffer_bytes)
            return data, sender
        except socket.timeout:
            return None
        except OSError:
            # 종료 중 소켓이 닫힌 경우 등. 호출자는 다음 루프에서 종료를 확인한다.
            return None

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
