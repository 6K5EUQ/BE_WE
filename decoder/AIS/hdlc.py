HDLC_FLAG = '01111110'


class FrameError(Exception):
    pass


def find_flag_positions(bits: str) -> list:
    positions = []
    i = 0
    while i <= len(bits) - 8:
        if bits[i:i+8] == HDLC_FLAG:
            positions.append(i)
            i += 8  
        else:
            i += 1
    return positions


def extract_frames(bits: str) -> list:
    positions = find_flag_positions(bits)
    frames = []

    for i in range(len(positions) - 1):
        start = positions[i] + 8
        end = positions[i + 1]
        frame_bits = bits[start:end]
        if len(frame_bits) >= 24:  
            frames.append(frame_bits)

    if positions:
        tail_start = positions[-1] + 8
        tail = bits[tail_start:]
        if len(tail) >= 24:
            frames.append(tail)

    return frames


def destuff(bits: str) -> str:
    result = []
    one_count = 0
    i = 0
    while i < len(bits):
        bit = bits[i]
        if bit == '1':
            one_count += 1
            result.append('1')
            if one_count == 5:
                i += 1
                if i < len(bits):
                    if bits[i] == '0':
                        one_count = 0  
                one_count = 0
        else:
            one_count = 0
            result.append('0')
        i += 1
    return ''.join(result)


def check_crc(frame_bits: str) -> tuple:
    if len(frame_bits) < 16:
        return frame_bits, False

    byte_count = len(frame_bits) // 8
    bytes_list = []
    for i in range(byte_count):
        byte_bits = frame_bits[i*8:(i+1)*8]
        val = int(byte_bits[::-1], 2)  
        bytes_list.append(val)

    crc = 0xFFFF
    for byte in bytes_list:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1

    valid = (crc == 0xF0B8)
    data_bits = frame_bits[:byte_count * 8 - 16] 
    return data_bits, valid
