"""Main module tests."""

import queue
import threading

import serial

from . import attitude_estimation, communication, config, main, testing_utils


class MockSerial(serial.SerialBase):
    def __init__(self):
        super().__init__()
        self.mosi_channel = queue.Queue()
        self.miso_channel = queue.Queue()

    def read(self, count: int):
        ret = bytes(self.mosi_channel.get() for _ in range(count))
        print(f"Device read: {ret}")
        return ret

    def write(self, data: bytes):
        print(f"Device wrote: {data}")
        for x in data:
            self.miso_channel.put(x)

    def reset_input_buffer(self):
        while not self.mosi_channel.empty():
            self.mosi_channel.get_nowait()


class MockSerialMaster(serial.SerialBase):
    def __init__(self, test_serial: MockSerial):
        super().__init__()
        self.test_serial = test_serial

    def read(self, count: int):
        return bytes(self.test_serial.miso_channel.get() for _ in range(count))

    def write(self, data: bytes):
        print(f"Master wrote: {data}")
        for x in data:
            self.test_serial.mosi_channel.put(x)

    def reset_input_buffer(self):
        while not self.test_serial.miso_channel.empty():
            self.test_serial.miso_channel.get_nowait()


class MasterEmulator:
    def __init__(
        self,
        ser: communication.PacketHandler,
    ):
        self.ser = ser

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        test_data_table = [
            (main.SetSettings, main.Settings(6, 10, 100, 2), main.ACK),
            (main.CalcTrajectory, main.EmptyMessage(), None),
        ]

        for command, tx_message, rx_expected in test_data_table:
            rx_message = self.transceive(command, tx_message)
            if rx_expected is not None:
                self.assert_response(command, rx_message, rx_expected, tx_message)

        self.transceive(
            main.SetAttitudeEstimationMode,
            main.AttitudeEstimationMode(
                attitude_estimation.AttitudeEstimationModeEnum.RECORD_DARKFRAME
            ),
        )

        response = self.transceive(main.GetStatus, main.EmptyMessage())
        assert (
            response.attitude_estimation_mode
            == attitude_estimation.AttitudeEstimationModeEnum.RECORD_DARKFRAME
        )
        import time

        while True:
            response = self.transceive(main.GetStatus, main.EmptyMessage())
            if (
                response.attitude_estimation_mode
                == attitude_estimation.AttitudeEstimationModeEnum.RECORD_DARKFRAME
            ):
                time.sleep(0.1)
            elif (
                response.attitude_estimation_mode
                == attitude_estimation.AttitudeEstimationModeEnum.IDLE
            ):
                break
            else:
                raise AssertionError()

        self.transceive(
            main.SetAttitudeEstimationMode,
            main.AttitudeEstimationMode(attitude_estimation.AttitudeEstimationModeEnum.RUNNING),
        )

        time.sleep(0.1)

        self.transceive(main.GetStars, main.EmptyMessage())

        self.send(main.Shutdown, main.EmptyMessage())

        print("success")

    def send(self, command: communication.Command, tx_message: communication.Message):
        assert command.request_type == type(tx_message)
        self.ser.write_cmd(command.cmd, tx_message.to_bytes())

    def transceive(self, command: communication.Command, tx_message: communication.Message):
        self.ser.reset_input_buffer()
        self.send(command, tx_message)
        cmd, payload = self.ser.read_cmd()
        assert cmd == command.cmd, (
            f"Command {command} with arguments {tx_message} "
            f"returned cmd id {cmd} instead of {command.cmd}"
        )
        rx_message = command.response_type.from_bytes(payload)
        return rx_message

    def assert_response(self, command, rx_message, rx_expected, tx_message):
        assert rx_message == rx_expected, (
            f"Command {command} with arguments {tx_message} "
            f"returned {rx_message} instead of {rx_expected}"
        )


def test_main():
    testing_utils.TestingMaterial(use_existing=True).patch_persistent()

    config.settings.shutdown_delay = 0.1

    device_serial = MockSerial()
    master_serial = MockSerialMaster(device_serial)
    packet_handler = communication.PacketHandler(master_serial)
    MasterEmulator(packet_handler)

    m = main.App(ser=device_serial)
    returncode = m()

    assert returncode == 5


if __name__ == "__main__":
    test_main()
