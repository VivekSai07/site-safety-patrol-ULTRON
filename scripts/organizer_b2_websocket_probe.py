"""Verify the noVNC websocket forwards a VNC greeting on private loopback."""

import base64
import hashlib
import socket


def main():
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    expected = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
    request = (
        "GET /websockify HTTP/1.1\r\nHost: 127.0.0.1:6081\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
        "Sec-WebSocket-Protocol: binary\r\n\r\n"
    )
    with socket.create_connection(("127.0.0.1", 6081), timeout=10) as connection:
        connection.settimeout(10)
        connection.sendall(request.encode())
        response = b""
        while b"\r\n\r\n" not in response:
            data = connection.recv(4096)
            if not data:
                raise RuntimeError("websocket closed during handshake")
            response += data
        headers, payload = response.split(b"\r\n\r\n", 1)
        if not headers.startswith(b"HTTP/1.1 101") or expected.encode() not in headers:
            raise RuntimeError("websocket upgrade failed")
        while len(payload) < 14:
            data = connection.recv(4096)
            if not data:
                raise RuntimeError("websocket closed before VNC greeting")
            payload += data
        if payload[0] != 0x82 or payload[1] != 12 or payload[2:14] != b"RFB 003.008\n":
            raise RuntimeError("websocket did not forward a VNC greeting")
        print("viewer websocket: HTTP 101, VNC RFB 3.8 greeting")


if __name__ == "__main__":
    main()
