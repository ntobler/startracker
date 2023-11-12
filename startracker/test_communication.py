from . import communication


def test_calc_crc():
    assert communication._calc_crc(b"123456789") == 0x29B1
    assert communication._calc_crc(b"foo") == 0x630A
    assert communication._calc_crc(b"") == 0xFFFF
