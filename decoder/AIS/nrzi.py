def nrzi_decode(bits: str, initial: str = '0') -> str:
    if not bits:
        return ''
    previous = initial
    decoded = []
    for b in bits:
        if b == previous:
            decoded.append('1')  
        else:
            decoded.append('0') 
        previous = b
    return ''.join(decoded)
