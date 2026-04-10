#!/usr/bin/env python3
"""Verify AIS from WAV IQ recording (dem_worker output)."""
import sys, struct, math, wave

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
        print("Usage: python3 verify_wav.py <wav_file>")
        return

    fname = sys.argv[1]
    wf = wave.open(fname, 'rb')
    sr = wf.getframerate()
    nch = wf.getnchannels()
    sw = wf.getsampwidth()
    nframes = wf.getnframes()
    raw = wf.readframes(nframes)
    wf.close()

    print(f"WAV: {sr} Hz, {nch} ch, {sw*8} bit, {nframes} frames, {nframes/sr:.2f}s")

    # Parse 16-bit stereo (I, Q)
    n = nframes
    I = [0.0] * n
    Q = [0.0] * n
    for i in range(n):
        off = i * nch * sw
        iv = struct.unpack_from('<h', raw, off)[0]
        qv = struct.unpack_from('<h', raw, off + sw)[0]
        I[i] = iv / 32768.0
        Q[i] = qv / 32768.0

    # Power
    power = sum(ii*ii + qq*qq for ii, qq in zip(I, Q)) / n
    print(f"Mean IQ power: {10*math.log10(power+1e-30):.1f} dB")
    print(f"I range: [{min(I):.6f}, {max(I):.6f}]")
    print(f"Q range: [{min(Q):.6f}, {max(Q):.6f}]")

    # Power envelope (1ms windows)
    win = max(1, sr // 1000)
    powers = []
    for i in range(0, n - win, win):
        p = sum(I[j]*I[j] + Q[j]*Q[j] for j in range(i, i+win)) / win
        powers.append(p)

    if powers:
        pmax = max(powers)
        pmin = min(powers)
        pmean = sum(powers) / len(powers)
        print(f"Power envelope: min={10*math.log10(pmin+1e-30):.1f} dB, "
              f"max={10*math.log10(pmax+1e-30):.1f} dB, "
              f"mean={10*math.log10(pmean+1e-30):.1f} dB")

        # Find bursts (power > mean + 6dB)
        thresh = pmean * 4  # +6 dB
        burst_count = 0
        in_burst = False
        for p in powers:
            if p > thresh and not in_burst:
                burst_count += 1
                in_burst = True
            elif p < thresh:
                in_burst = False
        print(f"Detected bursts (>{10*math.log10(thresh+1e-30):.1f} dB): {burst_count}")

    # FM discriminator
    disc = [0.0] * n
    for i in range(1, n):
        cross = I[i]*Q[i-1] - Q[i]*I[i-1]
        dot   = I[i]*I[i-1] + Q[i]*Q[i-1]
        disc[i] = math.atan2(cross, dot + 1e-30)

    # DC removal
    dc = sum(disc) / n
    disc = [d - dc for d in disc]

    rms = math.sqrt(sum(d*d for d in disc) / n)
    print(f"Disc RMS: {rms:.6f}")

    # Histogram
    mn = min(disc)
    mx = max(disc)
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

    for polarity in [1, -1]:
        d = [x * polarity for x in disc]
        bits = []
        phase = 0.0
        for i in range(n):
            phase += 1.0
            if phase >= sps:
                phase -= sps
                bits.append(1 if d[i] > 0 else 0)

        # NRZI
        nrz = []
        prev = 0
        for b in bits:
            nrz.append(1 if b == prev else 0)
            prev = b

        flag_cnt = 0
        frame_cnt = 0
        crc_fail = 0
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
                            reordered = []
                            for bi in range(min(plen,6)):
                                byte_idx = bi // 8
                                bit_idx = bi % 8
                                reordered.append(frame_bits[byte_idx*8 + (7-bit_idx)])
                            bitstr = ''.join(str(b) for b in reordered)
                            msg_type = int(bitstr, 2) if bitstr else 0
                            print(f"  [pol={polarity:+d}] CRC OK! plen={plen} type={msg_type} fcs={calcfcs:04x}")
                        else:
                            crc_fail += 1
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

        print(f"Polarity {polarity:+d}: flags={flag_cnt}, crc_fail={crc_fail}, decoded={frame_cnt}")

if __name__ == '__main__':
    main()
