"""Module to handle serial communication"""

import abc

import serial
import serial.serialutil

from typing import Tuple, Sequence, Type


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


class Message(abc.ABC):
    """Base class for packet messages."""

    @classmethod
    @abc.abstractmethod
    def from_bytes(cls, payload: bytes):
        pass

    @abc.abstractmethod
    def to_bytes(self) -> bytes:
        ...


class EmptyMessage(Message):
    @classmethod
    def from_bytes(cls, payload: bytes):
        return cls()

    def to_bytes(self) -> bytes:
        return b""


class Command(abc.ABC):
    """Base class for communication commands."""

    cmd: int
    payload_type: Type[Message] = Message

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
