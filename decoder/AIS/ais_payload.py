
def reorder_bits(data_bits: str) -> str:
    result = []
    for i in range(0, len(data_bits), 8):
        byte_chunk = data_bits[i:i+8]
        result.append(byte_chunk[::-1])
    return ''.join(result)


def get_uint(bits: str, start: int, length: int) -> int:
    return int(bits[start:start+length], 2)


def get_int(bits: str, start: int, length: int) -> int:
    val = get_uint(bits, start, length)
    if val >= (1 << (length - 1)):
        val -= (1 << length)
    return val


def sixbit_to_char(val: int) -> str:
    if val < 32:
        return chr(val + 64)
    else:
        return chr(val)


def get_string(bits: str, start: int, length: int) -> str:
    chars = []
    for i in range(start, start + length, 6):
        sixbit = int(bits[i:i+6], 2)
        chars.append(sixbit_to_char(sixbit))
    return ''.join(chars).rstrip('@')


def get_bool(bits: str, start: int) -> bool:
    return bits[start] == '1'
