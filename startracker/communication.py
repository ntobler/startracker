"""Module to handle serial communication"""

import abc
import dataclasses
import struct
import pathlib

import numpy as np

import serial
import serial.serialutil

from typing import Dict, Optional, Tuple, Sequence, Type, Union


class CommunicationTimeoutException(Exception):
    """Serial command reached timeout"""

    pass


def _calc_crc(data: bytes) -> int:
    """
    Calculate CRC-16/CCITT-FALSE of the data.

    Poly=0x1021, Init=0xFFFF, RefIn=False, RefOut=False, XorOut=0x0000

    Args:
        data: input data

    Returns:
        int: crc of the data
    """
    crc = 0xFFFF
    for x in data:
        crc ^= x << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc * 2) ^ 0x1021
            else:
                crc = crc * 2
    return crc & 0xFFFF


class PacketHandler(serial.Serial):
    """
    Receive and send protocol packets.

    Properties:
    - Big endian

    Packet structure
    - 1 cmd_id byte
    - 1 length byte, evaluates to n
    - n payload bytes
    - 2 crc bytes (crc16)
    """

    def __init__(self, ser: serial.Serial):
        self._ser = ser

    def read_cmd(self) -> Tuple[int, bytes]:
        """
        Read serial command from the input buffer.

        Raises:
            CommunicationTimeoutException if timeouts happen.

        Returns:
            Tuple[int, bytes]:
                - Command id
                - Payload bytes
        """

        timeout = serial.serialutil.Timeout(self._ser._timeout)
        # read cmd and length bytes
        c = self._ser.read(2)
        if timeout.expired():
            raise CommunicationTimeoutException()
        cmd, length = c

        # read payload and crc using length
        payload_and_crc = self._ser.read(length + 2)
        if timeout.expired():
            raise CommunicationTimeoutException()
        payload = payload_and_crc[:-2]
        crc = payload_and_crc[-2:]

        # check crc
        assert crc == _calc_crc(payload).to_bytes(2, "big")

        return cmd, payload

    def write_cmd(self, cmd: int, payload: bytes):
        """
        Send serial command.

        Args:
            cmd: command identifier
            payload: command payload.
        """

        length = len(payload)

        assert length < 256, "length does not fit in protocol"
        assert cmd < 255, "cmd is too high in value"

        crc = _calc_crc(payload).to_bytes(2, "big")

        # combine length and cmd as they share a common byte
        length_cmd = bytes((cmd, length))

        self._ser.write(length_cmd + payload + crc)


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


import enum


class EnumField(Field):
    """Message field for enums."""

    def __init__(self, enum_type: enum.Enum, dtype="uint8", desc: Optional[str] = None):
        super().__init__(dtype, desc=desc)
        self.default = enum_type(0)
        self._enum_type = enum_type

    def parse(self, value):
        return self._enum_type(value)

    def format(self, value):
        return int(value.value)

    def c_definition(self, name: str):
        c_code = f"{self._enum_type.__name__} {name};"
        if self.desc is not None:
            c_code += f" // {self.desc}"
        c_code += "\n"
        return c_code

    def generate_c_code(self, enum_args: str, indent: int):
        c_code = f"typedef enum {self._enum_type.__name__} : {self.c_type} {{\n"
        for k in self._enum_type:
            c_code += " " * indent + f"{k.name} = {k.value},\n"
        c_code += f"}} {self._enum_type.__name__};"
        return c_code


class StructField(Field):
    """Message field for custom datatypes."""

    def __init__(self, dtype: Type["Message"], desc: Optional[str] = None):
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
        return np.asarray(value, dtype=self._numpy_dtype).tobytes()


class Message:
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
    def generate_c_code(cls, struct_args: str, indent: int):
        c_code = f"typedef struct {struct_args} {{\n"
        for k, v in cls._fields.items():
            c_code += " " * indent + v.c_definition(k)
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
        bases=(Message,),
        eq=False,
    )


def gen_code_with_dependencies(
    messages: Sequence[Type[Message]],
    h_file: Union[pathlib.Path, str],
    indent: int = 4,
    skip_file_write: bool = False,
) -> str:
    """Export a .h file of the message definitions."""
    h_file = pathlib.Path(h_file)

    dependencies = list(messages)

    def check_msg(msg):
        for v in msg._fields.values():
            if type(v.dtype) != str:
                dependencies.append(v.dtype)
                check_msg(v.dtype)
            elif isinstance(v, EnumField):
                dependencies.append(v)

    for m in messages:
        check_msg(m)

    deps = list(dict.fromkeys(reversed(dependencies)))

    struct_args = "__attribute__((__packed__))"

    code = []
    for d in deps:
        code.append(d.generate_c_code(struct_args, indent))
    code = "\n\n".join(code)

    name = h_file.stem.upper()

    all_code = (
        "//Autogenerated code\n\n"
        f"#ifndef {name}_H_\n"
        f"#define {name}_H_\n"
        "\n"
        "#ifdef __cplusplus\n"
        'extern "C" {\n'
        "#endif\n"
        "\n"
        "#include <stdint.h>\n"
        "\n"
        f"{code}"
        "\n\n"
        "#ifdef __cplusplus\n"
        "}\n"
        "#endif\n"
        "\n"
        f"#endif /* {name}_H_ */\n"
    )

    if not skip_file_write:
        with open(h_file, "w") as f:
            f.write(all_code)

    return all_code


class Command(abc.ABC):
    """Base class for communication commands."""

    cmd: int
    argument_type: Type[Message] = Message

    @abc.abstractmethod
    def execute(self, payload: bytes) -> bytes:
        ...


class CommandHandler:
    """Execute received commands."""

    def __init__(
        self,
        serial: PacketHandler,
        commands: Sequence[Command],
        default_message: Message,
    ):
        self.serial = serial
        assert len({c.cmd for c in commands}) == len(
            commands
        ), "Commands are not unique."
        self._commands = {c.cmd: c for c in commands}
        self._default_message = default_message

    def run_indefinitely(self):
        """Handle incomming commands indefinitely."""
        while True:
            try:
                cmd_id, payload = self.serial.read_cmd()
                command = self._commands.get(cmd_id)
                if command is None:
                    self.serial.write_cmd(cmd_id, self._default_message.to_bytes())
                else:
                    self.serial.write_cmd(cmd_id, command.execute(payload))
            except CommunicationTimeoutException:
                pass