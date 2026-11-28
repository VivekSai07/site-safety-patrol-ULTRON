"""Capture one VNC framebuffer as PNG on stdout without creating container files."""

import base64
import io
import socket
import struct
import sys

from PIL import Image


def read_exact(connection, count):
    data = bytearray()
    while len(data) < count:
        part = connection.recv(count - len(data))
        if not part:
            raise RuntimeError("VNC connection closed")
        data.extend(part)
    return bytes(data)


def main():
    with socket.create_connection(("127.0.0.1", 5901), timeout=15) as connection:
        connection.settimeout(15)
        version = read_exact(connection, 12)
        if version != b"RFB 003.008\n":
            raise RuntimeError(f"unsupported VNC version: {version!r}")
        connection.sendall(version)
        count = read_exact(connection, 1)[0]
        if 1 not in read_exact(connection, count):
            raise RuntimeError("VNC server does not offer unauthenticated local access")
        connection.sendall(b"\x01")
        if read_exact(connection, 4) != b"\0\0\0\0":
            raise RuntimeError("VNC authentication failed")
        connection.sendall(b"\x01")
        width, height, _, name_length = struct.unpack(">HH16sI", read_exact(connection, 24))
        read_exact(connection, name_length)
        pixel_format = struct.pack(">BBBBHHHBBBxxx", 32, 24, 0, 1, 255, 255, 255, 16, 8, 0)
        connection.sendall(b"\0\0\0\0" + pixel_format)
        connection.sendall(struct.pack(">BBHi", 2, 0, 1, 0))
        connection.sendall(struct.pack(">BBHHHH", 3, 0, 0, 0, width, height))
        if read_exact(connection, 1) != b"\0":
            raise RuntimeError("unexpected VNC message")
        _, rectangles = struct.unpack(">BH", read_exact(connection, 3))
        frame = Image.new("RGB", (width, height))
        for _ in range(rectangles):
            x, y, w, h, encoding = struct.unpack(">HHHHi", read_exact(connection, 12))
            if encoding != 0:
                raise RuntimeError(f"unexpected VNC encoding: {encoding}")
            rectangle = Image.frombytes("RGB", (w, h), read_exact(connection, w * h * 4), "raw", "BGRX")
            frame.paste(rectangle, (x, y))
        output = io.BytesIO()
        frame.save(output, format="PNG")
        sys.stdout.write(base64.b64encode(output.getvalue()).decode("ascii"))


if __name__ == "__main__":
    main()
