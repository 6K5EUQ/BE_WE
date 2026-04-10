#!/usr/bin/env python3
"""Verify AIS baseband IQ dump - no numpy required."""
import sys, struct, math

def crc_ccitt(data_bytes):
    crc = 0xFFFF
    for b in data_bytes:
        for j in range(8):
            if (crc ^ b) & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
            b >>= 1
    return crc ^ 0xFFFF

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 verify_baseband.py <raw_file> [sample_rate=76896]")
        return

    fname = sys.argv[1]
    sr = int(sys.argv[2]) if len(sys.argv) > 2 else 76896

    with open(fname, 'rb') as f:
        raw = f.read()
    n_floats = len(raw) // 4
    data = struct.unpack(f'{n_floats}f', raw)
    I = data[0::2]
    Q = data[1::2]
    n = len(I)
    dur = n / sr
    print(f"Loaded {n} IQ samples, sr={sr} Hz, duration={dur:.2f}s")

    # Signal power
    power = sum(i*i + q*q for i, q in zip(I, Q)) / n
    print(f"Mean power: {10*math.log10(power+1e-30):.1f} dB")

    # FM discriminator
    disc = [0.0] * n
    for i in range(1, n):
        cross = I[i]*Q[i-1] - Q[i]*I[i-1]
        dot   = I[i]*I[i-1] + Q[i]*Q[i-1]
        disc[i] = math.atan2(cross, dot + 1e-30)

    # DC removal
    dc = sum(disc) / n
    disc = [d - dc for d in disc]

    # Stats
    rms = math.sqrt(sum(d*d for d in disc) / n)
    mn = min(disc)
    mx = max(disc)
    print(f"Disc RMS: {rms:.6f}, min={mn:.6f}, max={mx:.6f}")

    # Histogram of disc values (check if bimodal = FSK signal)
    bins = [0]*20
    for d in disc:
        idx = int((d - mn) / (mx - mn + 1e-30) * 19)
        idx = max(0, min(19, idx))
        bins[idx] += 1
    print("Disc histogram:")
    mx_bin = max(bins)
    for i, b in enumerate(bins):
        val = mn + (mx - mn) * i / 19
        bar = '#' * int(b / mx_bin * 40)
        print(f"  {val:+.4f}: {bar} ({b})")

    baud = 9600
    sps = sr / baud
    print(f"\nSPS: {sps:.2f}")

    # Try both polarities
    for polarity in [1, -1]:
        d = [x * polarity for x in disc]

        # Fixed-rate bit extraction
        bits = []
        phase = 0.0
        for i in range(n):
            phase += 1.0
            if phase >= sps:
                phase -= sps
                bits.append(1 if d[i] > 0 else 0)

        # NRZI decode
        nrz = []
        prev = 0
        for b in bits:
            nrz.append(1 if b == prev else 0)
            prev = b

        # Find flags and decode
        flag_cnt = 0
        frame_cnt = 0
        crc_fail_cnt = 0
        sreg = 0
        inframe = False
        frame_bits = []
        ones = 0
        skip = False

        for bit in nrz:
            sreg = ((sreg >> 1) | (bit << 7)) & 0xFF
            if sreg == 0x7E:
                flag_cnt += 1
                if inframe and len(frame_bits) >= 72:
                    plen = len(frame_bits) - 16
                    if 56 <= plen <= 512:
                        nbytes = (plen + 7) // 8
                        data_bytes = bytearray(nbytes)
                        for bi in range(min(plen, nbytes*8)):
                            data_bytes[bi//8] |= (frame_bits[bi] << (bi % 8))
                        rxfcs = 0
                        for bi in range(16):
                            rxfcs |= (frame_bits[plen + bi] << bi)
                        calcfcs = crc_ccitt(bytes(data_bytes))
                        if calcfcs == rxfcs:
                            frame_cnt += 1
                            # Parse msg type
                            reordered = []
                            for bi in range(min(plen,6)):
                                byte_idx = bi // 8
                                bit_idx = bi % 8
                                reordered.append(frame_bits[byte_idx*8 + (7-bit_idx)])
                            bitstr = ''.join(str(b) for b in reordered)
                            msg_type = int(bitstr, 2) if bitstr else 0
                            print(f"  [pol={polarity:+d}] CRC OK! plen={plen} type={msg_type} fcs={calcfcs:04x}")
                        else:
                            crc_fail_cnt += 1
                inframe = True
                frame_bits = []
                ones = 0
                skip = False
                continue

            if not inframe:
                continue
            if skip:
                skip = False
                if bit != 0:
                    inframe = False
                    frame_bits = []
                continue
            if bit == 1:
                ones += 1
                if ones == 5:
                    skip = True
            else:
                ones = 0
            frame_bits.append(bit)

        print(f"Polarity {polarity:+d}: flags={flag_cnt}, crc_fail={crc_fail_cnt}, decoded={frame_cnt}")

if __name__ == '__main__':
    main()
