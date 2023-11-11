"""Main entrypoint for the startracker application."""

from dataclasses import dataclass

import serial

from . import trajectory
from . import communication

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
        cls(payload[0], payload[1], payload[2], payload[3])

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


class CalcTrajectory(communication.Command):
    """Calculate trajectory using current attitude."""

    cmd: int = 2
    payload_type: Type[communication.Message] = communication.EmptyMessage

    def __init__(self, trajectory_calculator: trajectory.TrajectoryCalculator):
        self._trajectory_calculator = trajectory_calculator

    def execute(self, payload: bytes) -> bytes:
        # TODO add logic
        return ACK.to_bytes()


class Quit(communication.Command):
    """Quit the application."""

    cmd: int = 3
    payload_type: Type[communication.Message] = communication.EmptyMessage

    def execute(self, payload: bytes) -> bytes:
        raise KeyboardInterrupt()


class App:
    """Production application class."""

    def __init__(self):
        # TODO pass parameters from config
        ms = trajectory.MotorSolver(50)
        tc = trajectory.TrajectoryCalculator(150, 10, ms)

        # All available commands
        self._commands = [
            SetSettings(),
            CalcTrajectory(tc),
            Quit(),
        ]

        # Run communication handler to process incoming commands.
        self._ser = serial.Serial()

    def __call__(self) -> int:
        """Run main loop of the command handler."""
        s = communication.PacketHandler(self._ser)
        com_handler = communication.CommandHandler(s, self._commands, Acknowledge(False))
        try:
            com_handler.run_indefinitely()
        except KeyboardInterrupt:
            pass
        return 0


def main() -> int:
    """Run the application."""
    return App()()


if __name__ == "__main__":
    raise SystemExit(main())
