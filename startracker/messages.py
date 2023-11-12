# TODO cleanup, maybe merge with communication

import dataclasses
import struct
import pathlib

import numpy as np

from typing import Dict, Optional, Sequence, Type


class Field:
    """Message field for base datatypes."""

    def __init__(self, dtype: str, desc: Optional[str] = None):
        self.dtype = dtype
        (
            self.struct_char,
            self.python_type,
            self.c_type,
            self.byte_size,
            self._numpy_dtype,
        ) = {
            "uint8": ("B", int, "uint8_t", 1, np.uint8),
            "int8": ("b", int, "int8_t", 1, np.int8),
            "uint16": ("H", int, "uint16_t", 2, np.uint16),
            "int16": ("h", int, "int16_t", 2, np.int16),
            "uint32": ("I", int, "uint32_t", 2, np.uint32),
            "int32": ("i", int, "int32_t", 2, np.int32),
            "uint64": ("L", int, "uint64_t", 2, np.uint64),
            "int64": ("l", int, "int64_t", 2, np.int64),
            "float32": ("f", float, "float", 4, np.float32),
            "float64": ("d", float, "double", 8, np.float64),
        }[
            dtype
        ]
        self.default = 0
        self.desc = desc

    def c_definition(self, name: str):
        c_code = f"{self.c_type} {name};"
        if self.desc is not None:
            c_code += f" // {self.desc}"
        c_code += "\n"
        return c_code

    def parse(self, value):
        return value

    def format(self, value):
        return value


class StructField(Field):
    """Message field for custom datatypes."""

    def __init__(self, dtype: Type["MessageBase"], desc: Optional[str] = None):
        self.dtype = dtype
        self.struct_char = f"{dtype.byte_size}s"
        self.python_type = dtype
        self.c_type = dtype.__name__
        self.byte_size = dtype.byte_size
        self.default = 0
        self.desc = desc

    def parse(self, value):
        return self.dtype.from_bytes(value)

    def format(self, value):
        return value.to_bytes()


class ArrayField(Field):
    """Message field for multidimensional arrays of base datatypes."""

    def __init__(self, dtype: str, shape: Sequence[int], desc: Optional[str] = None):
        super().__init__(dtype, desc)
        self.dtype = dtype
        self.byte_size = self.byte_size * np.prod(shape).item()
        self.struct_char = f"{self.byte_size}s"
        self.shape = shape

    def c_definition(self, name: str):
        shape = "".join(f"[{x}]" for x in self.shape)
        c_code = f"{self.c_type} {name}{shape};"
        if self.desc is not None:
            c_code += f" // {self.desc}"
        c_code += "\n"
        return c_code

    def parse(self, value):
        return np.frombuffer(value, dtype=self._numpy_dtype).reshape(self.shape)

    def format(self, value: np.ndarray):
        return value.tobytes()


class MessageBase:
    byte_size: int
    _format: str
    _fields: Dict[str, Field]

    @classmethod
    def from_bytes(cls, payload: bytes):
        values = struct.unpack(cls._format, payload)
        values = [f.parse(v) for f, v in zip(cls._fields.values(), values)]
        return cls(*values)

    def to_bytes(self) -> bytes:
        values = [
            f.format(v) for f, v in zip(self._fields.values(), self.__dict__.values())
        ]
        return struct.pack(self._format, *values)

    @classmethod
    def print_help(cls):
        print(cls.__name__)
        for k, v in cls._fields.items():
            print(f"- {k}: {v.dtype}")

    @classmethod
    def generate_c_code(cls, struct_args: str):
        c_code = f"typedef struct {struct_args} {{\n"
        for k, v in cls._fields.items():
            c_code += "  " + v.c_definition(k)
        c_code += f"}} {cls.__name__};"
        return c_code


def array_safe_eq(a, b) -> bool:
    """Check if a and b are equal, even if they are numpy arrays."""
    if a is b:
        return True
    if isinstance(a, np.ndarray) and isinstance(b, np.ndarray):
        return a.shape == b.shape and (a == b).all()
    try:
        return a == b
    except TypeError:
        return NotImplemented


def dc_eq(dc1, dc2) -> bool:
    """Check if two dataclasses which hold numpy arrays are equal."""
    if dc1 is dc2:
        return True
    if dc1.__class__ is not dc2.__class__:
        return False
    t1 = dataclasses.astuple(dc1)
    t2 = dataclasses.astuple(dc2)
    return all(array_safe_eq(a1, a2) for a1, a2 in zip(t1, t2))


def make_message(cls):
    """Compose a message dataclass."""
    fields: Dict[str, Field] = {}
    for attr_name in cls.__dict__:
        if not attr_name.startswith("_"):
            attr = getattr(cls, attr_name)

            if not callable(attr):
                fields[attr_name] = attr

    format = "<" + "".join([cls.__dict__[k].struct_char for k, v in fields.items()])

    byte_size = sum([v.byte_size for v in fields.values()])

    dataclass_fields = []
    for k, v in fields.items():
        dataclass_fields.append(
            (k, v.python_type, dataclasses.field(default=v.default))
        )

    return dataclasses.make_dataclass(
        cls.__name__,
        dataclass_fields,
        namespace={
            "byte_size": byte_size,
            "_format": format,
            "_fields": fields,
            "__eq__": dc_eq,
        },
        bases=(MessageBase,),
        eq=False,
    )


def gen_code_with_dependencies(
    messages: Sequence[Type[MessageBase]], c_file: pathlib.Path
):
    dependencies = list(messages)

    def check_msg(msg):
        for v in msg._fields.values():
            if type(v.dtype) != str:
                dependencies.append(v.dtype)
                check_msg(v.dtype)

    for m in messages:
        check_msg(m)

    deps = list(dict.fromkeys(reversed(dependencies)))

    struct_args = "__attribute__((__packed__))"

    code = []
    for d in deps:
        code.append(d.generate_c_code(struct_args))
    code = "\n\n".join(code)

    with open(c_file, "w") as f:
        f.write("//Autogenerated code\n\n")
        f.write("#include <stdint.h>\n")
        f.write("#include <stdbool.h>\n\n")
        f.write(code)
        f.write("\n")


def test():
    @make_message
    class Bar:
        a = Field("uint8", "a")

    @make_message
    class Foo:
        min_matches = Field("uint8", "Min matches required in the attitude estimation")
        attitude_estimation_timeout_ms = Field("uint8")
        exposure_ms = StructField(Bar)
        gain = Field("uint8")
        foo = Field("float32")
        baz = ArrayField("uint8", (2, 3))

    Foo(1, 2, 3)
    assert Foo(
        49, 50, Bar(51), 52, 5.0, 49 * np.ones((2, 3), dtype=np.uint8)
    ) == Foo.from_bytes(b"1234" + np.array(5.0, dtype=np.float32).tobytes() + b"111111")
    assert Foo.from_bytes(b"12345678123456").to_bytes() == b"12345678123456"

    print(Foo().attitude_estimation_timeout_ms)
    print(Foo.print_help())


def test2():
    @make_message
    class Acknowledge:
        ack = Field("uint8", "1 if acknowledge okay else 0")

    @make_message
    class Settings:
        min_matches = Field("uint8", "Min matches required in the attitude estimation")
        attitude_estimation_timeout_ms = Field("uint8")
        exposure_ms = Field("uint16")
        gain = Field("uint8")

    @make_message
    class Trajectory:
        start = Field("float32", "Start time in seconds")
        stop = Field("float32", "Stop time in seconds")
        coeffs = ArrayField("float32", (3, 4), "Polynomial coefficients")

    @make_message
    class Quaternion:
        x = Field("float32", "x part of the quaternion")
        y = Field("float32", "y part of the quaternion")
        z = Field("float32", "z part of the quaternion")
        w = Field("float32", "w part of the quaternion")

    @make_message
    class Status:
        attitude_estimation_mode = Field("uint8")
        current_number_of_matches = Field("uint16")
        average_number_of_matches = Field("uint16")
        quaternion = StructField(Quaternion, "curent quaternion")
        estimation_id = Field("uint16", "Id of the current attitude estimation sample")

    gen_code_with_dependencies(
        [Acknowledge, Settings, Trajectory, Status], pathlib.Path("out.c")
    )


if __name__ == "__main__":
    test2()
