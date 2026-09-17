# -*- coding: utf-8 -*-
"""Small UDP datagram sender used by MORAI egress adapters."""

import socket


class UdpSender(object):
    def __init__(self, destination_ip, port):
        self.destination_ip = str(destination_ip)
        self.port = int(port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._closed = False

    def send(self, data):
        if self._closed:
            raise RuntimeError("UDP sender is closed")
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("UDP payload must be bytes-like")
        return self.socket.sendto(bytes(data), (self.destination_ip, self.port))

    def close(self):
        if not self._closed:
            self._closed = True
            try:
                self.socket.close()
            except OSError:
                pass

    def __del__(self):
        self.close()
