#!/usr/bin/env python3
"""Dependency-free Modbus TCP smoke test for the POSIX demo or a target board."""

from __future__ import annotations

import socket
import struct
import sys
import time


def recv_exact(sock: socket.socket, count: int) -> bytes:
    chunks: list[bytes] = []
    remaining = count
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise RuntimeError("connection closed while receiving response")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def exchange(sock: socket.socket, transaction: int, pdu: bytes) -> bytes:
    request = struct.pack(">HHHB", transaction, 0, len(pdu) + 1, 1) + pdu
    sock.sendall(request)
    header = recv_exact(sock, 7)
    rx_transaction, protocol, length, unit = struct.unpack(">HHHB", header)
    assert rx_transaction == transaction
    assert protocol == 0
    assert unit == 1
    return recv_exact(sock, length - 1)


def connect_with_retry(host: str, port: int) -> socket.socket:
    deadline = time.monotonic() + 5.0
    last_error: OSError | None = None
    while time.monotonic() < deadline:
        try:
            return socket.create_connection((host, port), timeout=2.0)
        except OSError as exc:
            last_error = exc
            time.sleep(0.05)
    raise RuntimeError(f"could not connect to {host}:{port}: {last_error}")


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 1502

    with connect_with_retry(host, port) as sock:
        sock.settimeout(2.0)

        response = exchange(sock, 1, bytes.fromhex("10 0000 0002 04 1234 ABCD"))
        assert response == bytes.fromhex("10 0000 0002")

        response = exchange(sock, 2, bytes.fromhex("03 0000 0002"))
        assert response == bytes.fromhex("03 04 1234 ABCD")

        response = exchange(sock, 3, bytes.fromhex("0F 0000 0004 01 0D"))
        assert response == bytes.fromhex("0F 0000 0004")

        response = exchange(sock, 4, bytes.fromhex("01 0000 0004"))
        assert response == bytes.fromhex("01 01 0D")

        response = exchange(sock, 5, bytes.fromhex("03 00FF 0002"))
        assert response == bytes.fromhex("83 02")

    print(f"Modbus TCP smoke test: PASS ({host}:{port})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
