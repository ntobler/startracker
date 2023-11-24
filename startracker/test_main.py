import threading
import queue
import inspect

import serial

from . import main
from . import communication

from typing import Sequence, Tuple, Type, Union


class MockSerial(serial.Serial):
    def __init__(self, replaces: serial.Serial):
        self.mosi_channel = queue.Queue()
        self.miso_channel = queue.Queue()
        self._timeout = replaces._timeout

    def read(self, count: int):
        ret = bytes(self.mosi_channel.get() for _ in range(count))
        print(f"Device read: {ret}")
        return ret

    def write(self, data: bytes):
        print(f"Device wrote: {data}")
        for x in data:
            self.miso_channel.put(x)


class MockSerialMaster(serial.Serial):
    def __init__(self, test_serial: MockSerial):
        self.test_serial = test_serial
        self._timeout = test_serial._timeout

    def read(self, count: int):
        return bytes(self.test_serial.miso_channel.get() for _ in range(count))

    def write(self, data: bytes):
        print(f"Master wrote: {data}")
        for x in data:
            self.test_serial.mosi_channel.put(x)


class MasterEmulator:
    def __init__(
        self,
        test_data: Sequence[
            Tuple[
                communication.Command,
                communication.Message,
                Union[None, communication.Message, Type[communication.Message]],
            ],
        ],
        ser: communication.PacketHandler,
    ):
        self.ser = ser
        self.test_data = test_data

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        for command, tx_message, rx_expected in self.test_data:
            assert command.argument_type == type(tx_message)
            self.ser.write_cmd(command.cmd, tx_message.to_bytes())

            if rx_expected is not None:
                cmd, payload = self.ser.read_cmd()
                assert cmd == command.cmd, (
                    f"Command {command} with arguments {tx_message} "
                    f"returned cmd id {cmd} instead of {command.cmd}"
                )

                if inspect.isclass(rx_expected):
                    rx_message = rx_expected.from_bytes(payload)
                    assert rx_expected == type(rx_message)
                else:
                    rx_message = type(rx_expected).from_bytes(payload)
                    assert rx_message == rx_expected, (
                        f"Command {command} with arguments {tx_message} "
                        f"returned {rx_message} instead of {rx_expected}"
                    )


def test_main():
    m = main.App(False)

    test_data_table = [
        (main.SetSettings, main.Settings(6, 10, 100, 2), main.ACK),
        (main.CalcTrajectory, main.EmptyMessage(), main.Trajectory),
        (main.Quit, main.EmptyMessage(), None),
    ]

    device_serial = MockSerial(m._ser)
    master_serial = MockSerialMaster(device_serial)
    packet_handler = communication.PacketHandler(master_serial)
    MasterEmulator(test_data_table, packet_handler)

    m._ser = device_serial
    m()


if __name__ == "__main__":
    test_main()
