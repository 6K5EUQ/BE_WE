
import sys
import argparse
import re

from nrzi import nrzi_decode
from hdlc import extract_frames, destuff, check_crc, FrameError
from ais_payload import reorder_bits
from message_types import parse_message

BANNER = """\
==================================================
                 AIS Signal Decoder
=================================================="""


def parse_input(raw: str) -> tuple:
    # Accepts binary, hex, or hex with 0x prefix.
    # Space, colon, hyphen separators are stripped.
    cleaned = re.sub(r'[\s:\-]', '', raw)

    if not cleaned:
        raise ValueError("Empty input")

    if cleaned.lower().startswith('0x'):
        cleaned = cleaned[2:]

    hex_chars = set('0123456789abcdefABCDEF')
    bin_chars = set('01')

    is_all_hex = all(c in hex_chars for c in cleaned)
    is_all_bin = all(c in bin_chars for c in cleaned)

    if is_all_bin and not is_all_hex:
        return cleaned, 'binary'

    if is_all_hex and not is_all_bin:
        if len(cleaned) % 2 != 0:
            raise ValueError(f"Odd hex length ({len(cleaned)} chars). Input must be byte-aligned.")
        bits = bin(int(cleaned, 16))[2:].zfill(len(cleaned) * 4)
        return bits, 'hex'

    if is_all_bin:
        return cleaned, 'binary'

    raise ValueError(
        f"Invalid input. Only binary (0/1) or hex (0-9, a-f) allowed.\n"
        f"  Got: {cleaned[:32]}..."
    )


def decode_bitstream(raw_bits: str, skip_nrzi: bool = False, force_decode: bool = False) -> list:
    # Try both NRZI initial states since we don't know where in the stream we started.
    # Only one of the two will produce CRC-valid frames.
    results = []

    if skip_nrzi:
        candidates = [raw_bits]
    else:
        candidates = [
            nrzi_decode(raw_bits, initial='0'),
            nrzi_decode(raw_bits, initial='1'),
        ]

    for decoded_bits in candidates:
        frames = extract_frames(decoded_bits)

        for frame_bits in frames:
            try:
                destuffed = destuff(frame_bits)
            except FrameError as e:
                continue

            data_bits, crc_valid = check_crc(destuffed)

            if not data_bits:
                continue

            if not crc_valid and not force_decode:
                continue

            reordered = reorder_bits(data_bits)
            fields = parse_message(reordered)
            results.append((fields, crc_valid))

    # Both NRZI candidates can produce the same frame, deduplicate.
    seen = set()
    unique = []
    for fields, crc_valid in results:
        key = str(fields)
        if key not in seen:
            seen.add(key)
            unique.append((fields, crc_valid))

    return unique


def format_output(fields: dict, crc_valid: bool) -> str:
    lines = []
    msg_type = fields.get("Message Type", "Unknown")
    lines.append("=" * 50)
    lines.append(f"  AIS Message: {msg_type}")
    lines.append(f"  CRC: {'Good' if crc_valid else 'Bad'}")
    lines.append("=" * 50)

    for key, value in fields.items():
        if key == "Message Type":
            continue
        lines.append(f"  {key:20s}: {value}")


    lines.append("")
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='AIS Signal Decoder',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
input formats:
  binary  string of 0s and 1s  e.g. 110011001100...
  hex     hex string            e.g. CCCCFE9A... or 0xCCCC...
  spaces, colons, hyphens as separators are allowed

examples:
  python ais_decoder.py                      interactive mode
  python ais_decoder.py "CCCCCCFE9AE4..."    hex input
  python ais_decoder.py "110011001100..."    binary input
  python ais_decoder.py --no-crc "CCCC..."  decode even if CRC fails
  python ais_decoder.py --skip-nrzi "..."   skip NRZI decoding
        """)
    parser.add_argument('bitstream',    nargs='?',           help='bitstream input (omit for interactive mode)')
    parser.add_argument('--no-crc',     action='store_true', help='decode even if CRC fails')
    parser.add_argument('--skip-nrzi',  action='store_true', help='skip NRZI decoding')

    args = parser.parse_args()

    def run_decode(raw_input: str) -> bool:
        try:
            bits, fmt = parse_input(raw_input)
        except ValueError as e:
            print(f"Input error: {e}\n", file=sys.stderr)
            return False


        results = decode_bitstream(bits, skip_nrzi=args.skip_nrzi, force_decode=args.no_crc)
        if not results:
            print("No valid AIS frame found.")
            print("Try --no-crc or check your input.\n")
            return False

        total = len(results)
        for i, (fields, crc_valid) in enumerate(results, 1):
            if total > 1:
                print(f"  -- Frame {i} of {total} --")
            print(format_output(fields, crc_valid))
        return True

    if args.bitstream:
        success = run_decode(args.bitstream.strip())
        sys.exit(0 if success else 1)
    else:
        print(BANNER)
        print()

        while True:
            try:
                raw = input("Input > ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nBye.")
                break

            if not raw:
                continue
            if raw.lower() in ('q', 'quit', 'exit'):
                print("Bye.")
                break

            run_decode(raw)


if __name__ == '__main__':
    main()
