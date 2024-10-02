"""Communication module tests."""

import enum

import numpy as np

from startracker import communication


def test_calc_crc():
    assert communication.calc_crc(b"123456789") == 0x29B1
    assert communication.calc_crc(b"foo") == 0x630A
    assert communication.calc_crc(b"") == 0xFFFF


def test_messages():
    @communication.make_message
    class Bar:
        a = communication.Field("uint8", "a")

    class MyEnum(enum.Enum):
        A = 0
        B = 1
        C = 2

    @communication.make_message
    class Foo:
        min_matches = communication.Field(
            "uint8", "Min matches required in the attitude estimation"
        )
        attitude_estimation_timeout_ms = communication.Field("uint8")
        exposure_ms = communication.StructField(Bar)
        gain = communication.Field("uint8")
        foo = communication.Field("float32")
        baz = communication.ArrayField("uint8", (2, 3))
        test_enum = communication.EnumField(MyEnum, desc="Test enum")

    Foo(1, 2, 3)
    assert Foo(
        49, 50, Bar(51), 52, 5.0, 49 * np.ones((2, 3), dtype=np.uint8), MyEnum.C
    ) == Foo.from_bytes(b"1234" + np.array(5.0, dtype=np.float32).tobytes() + b"111111" + b"\x02")
    assert Foo.from_bytes(b"12345678123456\x02").to_bytes() == b"12345678123456\x02"

    assert type(Foo().test_enum) == MyEnum

    print(Foo().attitude_estimation_timeout_ms)
    print(Foo.print_help())

    code = communication.gen_code_with_dependencies([Foo], "out.h", skip_file_write=True)
    print(code)
