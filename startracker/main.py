"""Main entrypoint for the startracker application."""

import sys
from dataclasses import dataclass
import logging

import serial
import numpy as np

from . import trajectory
from . import communication
from . import attitude_estimation
from . import config

from typing import Type


@dataclass
class Acknowledge(communication.Message):
    okay: bool

    @classmethod
    def from_bytes(cls, payload: bytes):
        return cls(bool(payload[0]))

    def to_bytes(self) -> bytes:
        return bytes([int(self.okay)])


ACK = Acknowledge(True)
NACK = Acknowledge(False)


@dataclass
class Settings(communication.Message):
    min_matches: int
    attitude_estimation_timeout_ms: int
    exposure_ms: int
    gain: int

    @classmethod
    def from_bytes(cls, payload: bytes):
        return cls(payload[0], payload[1], payload[2], payload[3])

    def to_bytes(self) -> bytes:
        return bytes(
            (
                self.min_matches,
                self.attitude_estimation_timeout_ms,
                self.exposure_ms,
                self.gain,
            )
        )


class SetSettings(communication.Command):
    """Set star tracker settings."""

    cmd: int = 1
    payload_type: Type[communication.Message] = Settings

    def execute(self, payload: bytes) -> bytes:
        settings = self.payload_type.from_bytes(payload)
        # TODO add logic
        return ACK.to_bytes()


@dataclass
class Trajectory(communication.Message):
    start: float
    stop: float
    coeffs: np.ndarray

    @classmethod
    def from_bytes(cls, payload: bytes):
        data = np.frombuffer(payload, dtype=np.float32)
        return cls(data[0].item(), data[1].item(), data[2:])

    def to_bytes(self) -> bytes:
        data = np.concatenate(
            (
                np.asarray((self.start, self.stop), dtype=np.float32),
                np.asarray(self.coeffs, dtype=np.float32).ravel(),
            ),
            axis=0,
        )
        return data.tobytes()


class CalcTrajectory(communication.Command):
    """Calculate trajectory using current attitude."""

    cmd: int = 2
    payload_type: Type[communication.Message] = communication.EmptyMessage

    def __init__(
        self,
        attitude_filter: attitude_estimation.AttitudeFilter,
        trajectory_calculator: trajectory.TrajectoryCalculator,
    ):
        self._attitude_filter = attitude_filter
        self._trajectory_calculator = trajectory_calculator

    def execute(self, payload: bytes) -> bytes:
        az_el = self._attitude_filter.get_azimuth_elevation()

        if az_el is None:
            return NACK.to_bytes()

        az, el = az_el
        polynom_trajectory = self._trajectory_calculator(az, el)
        message = Trajectory(
            polynom_trajectory.start,
            polynom_trajectory.stop,
            polynom_trajectory.position_coeffs.tolist(),
        )
        return message.to_bytes()


class Quit(communication.Command):
    """Quit the application."""

    cmd: int = 3
    payload_type: Type[communication.Message] = communication.EmptyMessage

    def execute(self, payload: bytes) -> bytes:
        raise KeyboardInterrupt()


class App:
    """Production application class."""

    def __init__(self):
        logging.info("Starting application")

        s = config.settings

        config.check_validation_errors()

        # TODO pass parameters from config
        # TODO use config manager e.g. dynaconf
        ms = trajectory.MotorSolver(
            s.hardware.motor_dists_from_center,
            np.radians(s.hardware.motor_theta).item(),
            np.radians(s.hardware.motor_phi).item(),
        )
        tc = trajectory.TrajectoryCalculator(
            s.trajectory.max_seconds, s.trajectory.max_dist, ms
        )
        af = attitude_estimation.AttitudeFilter()

        # All available commands
        self._commands = [
            SetSettings(),
            CalcTrajectory(af, tc),
            Quit(),
        ]

        # Run communication handler to process incoming commands.
        self._ser = serial.Serial(config.settings.serial_port)

    def __call__(self) -> int:
        """Run main loop of the command handler."""
        s = communication.PacketHandler(self._ser)
        com_handler = communication.CommandHandler(
            s, self._commands, Acknowledge(False)
        )
        logging.info("Running")
        try:
            com_handler.run_indefinitely()
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            pass
        return 0


def main() -> int:
    """Run the application."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )
    e = None
    try:
        return App()()
    except Exception as e:
        logging.critical("Unhandled exception", exc_info=sys.exc_info())
        return -1


if __name__ == "__main__":
    raise SystemExit(main())
