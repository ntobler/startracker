"""Main entrypoint for the startracker application."""

import sys
import logging
import pathlib
import enum
import argparse
import subprocess

import serial
import numpy as np

from . import trajectory
from . import communication
from . import attitude_estimation
from . import config

from typing import Type


class AcknowledgeEnum(enum.Enum):
    NACK = 0
    ACK = 1


@communication.make_message
class Acknowledge:
    ack = communication.EnumField(
        AcknowledgeEnum, "uint8", "1 if acknowledge okay else 0"
    )


@communication.make_message
class Settings:
    min_matches = communication.Field(
        "uint8", "Min matches required in the attitude estimation"
    )
    attitude_estimation_timeout_ms = communication.Field("uint8")
    exposure_ms = communication.Field("uint16")
    gain = communication.Field("uint8")


@communication.make_message
class Trajectory:
    start = communication.Field("float32", "Start time in seconds")
    stop = communication.Field("float32", "Stop time in seconds")
    coeffs = communication.ArrayField("float32", (3, 4), "Polynomial coefficients")


@communication.make_message
class Quaternion:
    x = communication.Field("float32", "x part of the quaternion")
    y = communication.Field("float32", "y part of the quaternion")
    z = communication.Field("float32", "z part of the quaternion")
    w = communication.Field("float32", "w part of the quaternion")


class AttitudeEstimationMode(enum.Enum):
    OFF = 0
    ON = 1
    SINGLE = 2


@communication.make_message
class Status:
    attitude_estimation_mode = communication.EnumField(AttitudeEstimationMode, "uint8")
    current_number_of_matches = communication.Field("uint16")
    average_number_of_matches = communication.Field("uint16")
    quaternion = communication.StructField(Quaternion, "curent quaternion")
    estimation_id = communication.Field(
        "uint16", "Id of the current attitude estimation sample"
    )


@communication.make_message
class EmptyMessage:
    pass


def generate_code():
    """Utility function to generate code for the STM projet"""
    communication.gen_code_with_dependencies(
        [Acknowledge, Settings, Trajectory, Status], pathlib.Path("out.h")
    )


# Instantiate, as they are used frequently
ACK = Acknowledge(AcknowledgeEnum.ACK)
NACK = Acknowledge(AcknowledgeEnum.NACK)


class GetStatus(communication.Command):
    """Get the application status."""

    cmd: int = 1
    argument_type: Type[communication.Message] = Status

    def __init__(
        self,
        attitude_filter: attitude_estimation.AttitudeFilter,
        trajectory_calculator: trajectory.TrajectoryCalculator,
    ):
        self._attitude_filter = attitude_filter
        self._trajectory_calculator = trajectory_calculator

    def execute(self, payload: bytes) -> bytes:
        status = Status(
            attitude_estimation_mode=69,  # TODO implement
            current_number_of_matches=420,
            average_number_of_matches=314,
            quaternion=self._attitude_filter.attitude_quat,
            estimation_id=self._attitude_filter.estimation_id,
        )
        return status.to_bytes()


class SetSettings(communication.Command):
    """Set star tracker settings."""

    cmd: int = 2
    argument_type: Type[communication.Message] = Settings

    def execute(self, payload: bytes) -> bytes:
        settings = self.argument_type.from_bytes(payload)
        # TODO add logic
        return ACK.to_bytes()


class CalcTrajectory(communication.Command):
    """Calculate trajectory using current attitude."""

    cmd: int = 3
    argument_type: Type[communication.Message] = EmptyMessage

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


class ShutdownInterrupt(Exception):
    pass


class Quit(communication.Command):
    cmd: int = 4
    argument_type: Type[communication.Message] = EmptyMessage

    def __init__(self, enable_shutdown: bool):
        """
        Quit the application.

        Args:
            enable_shutdown: Application is allowed to shutdown the system
        """
        self._enable_shutdown = enable_shutdown

    def execute(self, payload: bytes) -> bytes:
        if self._enable_shutdown:
            raise ShutdownInterrupt()
        else:
            raise KeyboardInterrupt()


# TODO implement
# class SetAttitudeEstimationMode(AttitudeEstimationMode) returns (Acknowledge)
# class RecordDarkFrame(Empty) returns (Acknowledge)


class App:
    def __init__(self, enable_shutdown: bool):
        """
        Production application class.

        Args:
            enable_shutdown: Application is allowed to shutdown the system
        """
        logging.info("Starting application")

        s = config.settings

        config.check_validation_errors()

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
            GetStatus(af, tc),
            SetSettings(),
            CalcTrajectory(af, tc),
            Quit(enable_shutdown),
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
            logging.info("KeyboardInterrupt")
            pass
        except ShutdownInterrupt:
            logging.info("ShutdownInterrupt")
            subprocess.Popen("shutdown -h +1")
        return 0


def main() -> int:
    """Run the application."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-shutdown", action="store_true", default=False)
    args = parser.parse_args()

    try:
        return App(args.enable_shutdown)()
    except Exception:
        logging.critical("Unhandled exception", exc_info=sys.exc_info())
        return -1


if __name__ == "__main__":
    raise SystemExit(main())
