#!/usr/bin/env python3
"""AIS pipe decoder: C++ ais_worker로부터 CRC-검증된 프레임을 stdin으로 받아 파싱 후 stdout으로 출력.

프로토콜:
  IN:  FRAME <ch_idx> <nbits> <bit_string>\n
  OUT: 디코딩된 텍스트 블록\n---END---\n

bit_string은 C++에서 이미 MSB-first로 재정렬된 페이로드 비트.
(NRZI/HDLC/CRC/비트재정렬 처리 완료)
"""

import sys
import os
import time

from message_types import parse_message


def format_decoded(fields: dict, ch_idx: int, nbits: int) -> str:
    lines = []
    msg_type_str = fields.get("메시지 타입", fields.get("Message Type", "?"))
    mmsi = fields.get("MMSI", "?")

    ts = time.strftime("%H:%M:%S")
    lines.append(f"[{ts}] CH{ch_idx} | {msg_type_str} | MMSI {mmsi}")
    lines.append("-" * 50)

    for key, value in fields.items():
        if key in ("메시지 타입", "Message Type"):
            continue
        lines.append(f"  {key:20s}: {value}")

    lines.append("")
    return '\n'.join(lines)


def main():
    # line-buffered stdout
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        parts = line.split(None, 3)  # FRAME ch_idx nbits bit_string
        if len(parts) != 4 or parts[0] != 'FRAME':
            continue

        try:
            ch_idx = int(parts[1])
            nbits = int(parts[2])
            bit_string = parts[3]
        except (ValueError, IndexError):
            continue

        if len(bit_string) > nbits:
            bit_string = bit_string[:nbits]

        fields = parse_message(bit_string)
        output = format_decoded(fields, ch_idx, nbits)
        sys.stdout.write(output + '\n')
        sys.stdout.write('---END---\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
