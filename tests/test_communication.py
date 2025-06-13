"""Communication module tests."""

import enum

import numpy as np
import pytest

from startracker import communication


def test_calc_crc():
    assert communication.calc_crc(b"123456789") == 0x29B1
    assert communication.calc_crc(b"foo") == 0x630A
    assert communication.calc_crc(b"") == 0xFFFF


def test_messages():
    @communication.make_message
    class Bar(communication.Message):
        a = communication.NumberField("uint8", "a")
        b = communication.NumberField("int16", "b")

    class MyEnum(enum.Enum):
        A = 0
        B = 1
        C = 2

    @communication.make_message
    class Foo(communication.Message):
        min_matches = communication.NumberField(
            "uint8", "Min matches required in the attitude estimation"
        )
        attitude_estimation_timeout_ms = communication.NumberField("uint8")
        exposure_ms = communication.StructField(Bar)
        gain = communication.NumberField("uint8")
        foo = communication.NumberField("float32")
        baz = communication.ArrayField("uint8", (2, 3))
        test_enum = communication.EnumField(MyEnum, desc="Test enum")

    foo: Foo = Foo(49, 50, Bar(51, 52), 52, 5.0, 49 * np.ones((2, 3), dtype=np.uint8), MyEnum.C)
    foo_from_bytes = Foo.from_bytes(
        b"1234\x004" + np.array(5.0, dtype=np.float32).tobytes() + b"111111" + b"\x02"
    )

    assert foo == foo_from_bytes
    assert foo.min_matches == foo.min_matches
    assert Foo.from_bytes(b"1233345678123456\x02").to_bytes() == b"1233345678123456\x02"

    assert isinstance(foo.test_enum, MyEnum)
    assert isinstance(foo.gain, int)
    assert isinstance(foo.foo, float)

    print(Foo.print_help())

    code = communication.gen_code_with_dependencies([Foo], "out.h", skip_file_write=True)
    print(code)


if __name__ == "__main__":
    pytest.main([__file__])
