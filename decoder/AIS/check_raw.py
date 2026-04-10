#!/usr/bin/env python3
"""Check raw IQ dump for AIS signal presence using Goertzel."""
import sys, struct, math

def goertzel_power(samples_i, samples_q, freq_hz, sr):
    """Goertzel algorithm to measure power at specific frequency."""
    n = len(samples_i)
    k = int(0.5 + n * freq_hz / sr)
    w = 2.0 * math.pi * k / n
    coeff = 2.0 * math.cos(w)
    s0 = s1 = s2 = 0.0
    for i in range(n):
        x = samples_i[i] + samples_q[i] * 0  # Use I channel only for real Goertzel
        s0 = x + coeff * s1 - s2
        s2 = s1
        s1 = s0
    power = s1*s1 + s2*s2 - coeff*s1*s2
    return power / (n * n)

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 check_raw.py <raw_file> <sample_rate>")
        print("  e.g.: python3 check_raw.py /tmp/ais_raw_ch0.raw 61440000")
        print("        python3 check_raw.py /tmp/ais_dec_ch0.raw 76896")
        return

    fname = sys.argv[1]
    sr = int(sys.argv[2])

    with open(fname, 'rb') as f:
        raw = f.read()
    n_floats = len(raw) // 4
    data = struct.unpack(f'{n_floats}f', raw)
    I = list(data[0::2])
    Q = list(data[1::2])
    n = len(I)
    dur = n / sr
    print(f"File: {fname}")
    print(f"Samples: {n}, SR: {sr} Hz, Duration: {dur:.4f}s")

    # Overall stats
    power = sum(i*i + q*q for i, q in zip(I, Q)) / n
    print(f"Mean IQ power: {10*math.log10(power+1e-30):.1f} dB")
    print(f"I range: [{min(I):.6f}, {max(I):.6f}]")
    print(f"Q range: [{min(Q):.6f}, {max(Q):.6f}]")

    # Check specific frequencies using complex DFT (more accurate)
    print(f"\nSpectral power at key frequencies:")
    test_freqs = [0, 1000, 5000, 10000, 15000, 20000, 25000, 30000, -25000, -20000, -15000, -10000, -5000, -1000]
    if sr > 1000000:
        test_freqs = [0, 25000, 25100, 25200, 25300, -25000, -25100, -25200, -25300,
                      50000, -50000, 100000, -100000]

    # Use first 8192 samples for DFT
    N = min(n, 8192)
    for freq in sorted(test_freqs):
        if abs(freq) > sr/2:
            continue
        # Complex DFT at single frequency
        re = 0.0
        im = 0.0
        w = -2.0 * math.pi * freq / sr
        for k in range(N):
            ang = w * k
            c = math.cos(ang)
            s = math.sin(ang)
            x = I[k] + 1j * Q[k]
            re += I[k] * c - Q[k] * s
            im += I[k] * s + Q[k] * c
        mag = math.sqrt(re*re + im*im) / N
        db = 20*math.log10(mag + 1e-30)
        bar = '#' * max(0, int((db + 80) / 2))
        print(f"  {freq:+7d} Hz: {db:+6.1f} dB  {bar}")

if __name__ == '__main__':
    main()
