import logging

logger = logging.getLogger("utils")
logger.setLevel(logging.WARNING)

def int2hexstring(number: int, size: int = 4) -> bytes:
    """
    Returns 0-padded hex string without '0x' prefix in bytes format.

    Args:
        number (int): Value to convert to hex
        size (int): Size to complete the number of chars using padding

    Returns:
        bytes: utf-8 encoded hex string
    """
    encoded = f"{number:#0{size+2}x}"[2:].encode()
    if len(encoded) > size:
        logger.warning(f"Requested padding of size {size} but the number {number} is too big")
    return encoded

def hexstring2int(hex: str | bytes) -> int:
    """
    Returns the integer value of the hexadecimal string.

    Args:
        hex (str | bytes): _description_

    Returns:
        int: _description_
    """
    try:
        return int(hex, 16)
    except ValueError as ex:
        logger.error(f"hexstring2int got ValueError: {repr(ex)}")
        return -1
