"""Main module tests."""

import pathlib
import threading

import pytest
from typing_extensions import Self

from startracker import attitude_estimation, communication, config, main, testing_utils


class MasterEmulator:
    def __init__(
        self,
        ser: communication.PacketHandler,
    ):
        self.ser = ser

        self.thread = threading.Thread(target=self._run, daemon=False)

    def __enter__(self) -> Self:
        self.thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.thread.join()

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

        loop_broken = False
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
                loop_broken = True
                break
        assert loop_broken

        self.transceive(
            main.SetAttitudeEstimationMode,
            main.AttitudeEstimationMode(attitude_estimation.AttitudeEstimationModeEnum.RUNNING),
        )

        time.sleep(0.1)

        self.transceive(main.GetStars, main.EmptyMessage())

        self.send(main.Shutdown, main.EmptyMessage())

        print("success")

    def send(self, command: type[communication.Command], tx_message: communication.Message):
        """Send a command to the device."""
        assert command.request_type == type(tx_message)
        self.ser.write_cmd(command.cmd, tx_message.to_bytes())

    def transceive(
        self, command: type[communication.Command], tx_message: communication.Message
    ) -> communication.Message:
        """Send a command and receive the response."""
        self.ser.reset_input_buffer()
        self.send(command, tx_message)
        cmd, payload = self.ser.read_cmd()
        assert cmd == command.cmd, (
            f"Command {command} with arguments {tx_message} "
            f"returned cmd id {cmd} instead of {command.cmd}"
        )
        rx_message = command.response_type.from_bytes(payload)
        return rx_message

    def assert_response(self, command, rx_message, rx_expected, tx_message) -> None:
        """Assert that the response message is as expected."""
        assert rx_message == rx_expected, (
            f"Command {command} with arguments {tx_message} "
            f"returned {rx_message} instead of {rx_expected}"
        )


def test_main(tmp_path: pathlib.Path):
    testing_utils.patch_persistent(tmp_path, cal=True)

    config.settings.shutdown_delay = 0.1

    device_serial = testing_utils.MockSerial()
    master_serial = testing_utils.MockSerialMaster(device_serial)
    packet_handler = communication.PacketHandler(master_serial)
    with MasterEmulator(packet_handler), main.App(ser=device_serial) as app:
        returncode = app()

    assert returncode == 5


if __name__ == "__main__":
    pytest.main([__file__])
