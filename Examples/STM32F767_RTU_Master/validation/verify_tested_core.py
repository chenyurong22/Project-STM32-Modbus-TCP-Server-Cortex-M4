#!/usr/bin/env python3
"""Verify the portable files used by the externally tested RTU master target.

The Keil archive used CRLF line endings and the Git repository uses LF.
Hashes are calculated after CRLF/CR -> LF normalization.
"""

from __future__ import annotations

import hashlib
import sys
from pathlib import Path

EXPECTED = {'App/include/modbus.h': '89c42343b5549dd9fe077b1b2f36f091de0e2466d23aeb42292f2aad25f146c1', 'App/include/modbus_crc16.h': 'a1ac8a84d054da2be058f682eae45fe553c1b51cc46420c1fb7e4e594dbaeba0', 'App/include/modbus_pdu.h': '2d7a33782930bf1865f0ae2dd87c7a01b02e11eaf747b01f59bcc4c79a128bb8', 'App/include/modbus_protocol.h': 'dc7df5b1446fccbdc23efb9603c48e35602cbaf3c78c1209ee6ed9108fbc4b69', 'App/include/modbus_rtu.h': 'ea6199fdf8932ebb210a545ecf9abb8bdf8974a4ec3751f9bbac9c546536b652', 'App/include/modbus_rtu_master.h': '695c6eee6ccedb46407f97cbdc2916365303344a03f21d8b2b2f9d23fb292c5d', 'App/include/platform_port.h': '5217e32efa9ae722effa0ca26bd125553ffccd6e28643065c132c3cfb699efff', 'App/src/modbus.c': '3600b9bfd2e9f74546e22ba88e6b1104b304cf408b49ed958288b05b8306a882', 'App/src/modbus_crc16.c': '9b7530e77b65e11f4045da888505524dc894e9c6d2a4accac5124c98ae655d2f', 'App/src/modbus_protocol.c': 'fd3bde018ad251084295b325a8ddca40d9dc580c0e04bbd335564a3d0d0e628d', 'App/src/modbus_rtu.c': '69ab0ff4a21c4d31d7a10f6b8630831c28ab6d6406ee095161aa24e5865738f9', 'App/src/modbus_rtu_master.c': '4f9e296a8811a2b9f69c2fc4593ce9fff143e0f9b4346dbdde4405e66ca53eed'}


def normalized_sha256(path: Path) -> str:
    data = path.read_bytes()
    data = data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    failures = 0

    for relative, expected in EXPECTED.items():
        path = root / relative
        if not path.is_file():
            print(f"MISSING  {relative}")
            failures += 1
            continue

        actual = normalized_sha256(path)
        if actual == expected:
            print(f"OK       {relative}")
        else:
            print(f"DIFF     {relative}")
            print(f"         expected {expected}")
            print(f"         actual   {actual}")
            failures += 1

    if failures:
        print(f"RESULT: {failures} scoped file(s) differ from the tested source content.")
        return 1

    print("RESULT: all scoped portable files match the externally tested source content.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
