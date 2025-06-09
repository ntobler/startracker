"""Main entrypoint for the startracker application."""

import argparse
import enum
import logging
import sys
import time
from typing import Optional

import numpy as np
import serial
from typing_extensions import Never, Self, override

from startracker import attitude_estimation, communication, config, trajectory


class AcknowledgeEnum(enum.Enum):
    NACK = 0
    ACK = 1


@communication.make_message
class Acknowledge(communication.Message):
    ack = communication.EnumField(AcknowledgeEnum, "uint8", "1 if acknowledge okay else 0")

    def __init__(self, ack: AcknowledgeEnum):
        self.ack = ack


@communication.make_message
class Settings(communication.Message):
    min_matches = communication.NumberField(
        "uint8", "Min matches required in the attitude estimation"
    )
    attitude_estimation_timeout_ms = communication.NumberField("uint8")
    exposure_ms = communication.NumberField("uint16")
    gain = communication.NumberField("uint8")


@communication.make_message
class Trajectory(communication.Message):
    ack = communication.EnumField(
        AcknowledgeEnum,
        "uint8",
        "1 if the trajectory could be calculated successfully, otherwise 0",
    )
    start = communication.NumberField("float32", "Start time in seconds")
    stop = communication.NumberField("float32", "Stop time in seconds")
    coeffs = communication.ArrayField("float32", (3, 4), "Polynomial coefficients")


@communication.make_message
class Quaternion(communication.Message):
    x = communication.NumberField("float32", "x part of the quaternion")
    y = communication.NumberField("float32", "y part of the quaternion")
    z = communication.NumberField("float32", "z part of the quaternion")
    w = communication.NumberField("float32", "w part of the quaternion")


@communication.make_message
class AttitudeEstimationMode(communication.Message):
    mode = communication.EnumField(attitude_estimation.AttitudeEstimationModeEnum, "uint8")


@communication.make_message
class Status(communication.Message):
    attitude_estimation_mode = communication.EnumField(
        attitude_estimation.AttitudeEstimationModeEnum, "uint8"
    )
    current_number_of_matches = communication.NumberField("uint16")
    average_number_of_matches = communication.NumberField("uint16")
    quaternion = communication.StructField(Quaternion, "Current quaternion")
    estimation_id = communication.NumberField(
        "uint16", "Id of the current attitude estimation sample"
    )


MAX_STAR_COUNT = 32
"""Maximal number of stars in the star position list."""


@communication.make_message
class Stars(communication.Message):
    count = communication.NumberField("uint8", "Number of recognized stars")
    positions = communication.ArrayField(
        "uint8",
        (MAX_STAR_COUNT, 2),
        "x (0..127) and y (0..63) positions of recognized stars, shape=[32, 2]",
    )


@communication.make_message
class EmptyMessage(communication.Message):
    pass


# Instantiate, as they are used frequently
ACK = Acknowledge(ack=AcknowledgeEnum.ACK)
NACK = Acknowledge(ack=AcknowledgeEnum.NACK)


class GetStatus(communication.Command):
    """Get the application status."""

    cmd: int = 1
    request_type: type[communication.Message] = EmptyMessage
    response_type: type[communication.Message] = Status

    def __init__(
        self,
        attitude_filter: attitude_estimation.AttitudeFilter,
        trajectory_calculator: trajectory.TrajectoryCalculator,
        image_acquisitioner: attitude_estimation.ImageAcquisitioner,
    ):
        self._attitude_filter = attitude_filter
        self._trajectory_calculator = trajectory_calculator
        self._image_acquisitioner = image_acquisitioner

    @override
    def execute(self, request: EmptyMessage) -> Status:
        _ = request
        quat = self._attitude_filter.attitude_quat
        if quat is None:
            quat = np.full((4,), np.nan, np.float32)
        status = Status(
            attitude_estimation_mode=self._image_acquisitioner.mode,
            current_number_of_matches=420,  # TODO implement
            average_number_of_matches=314,  # TODO implement
            quaternion=Quaternion(quat[0], quat[1], quat[2], quat[3]),
            estimation_id=self._attitude_filter.estimation_id,
        )
        return status


class SetSettings(communication.Command):
    """Set star tracker settings."""

    cmd: int = 2
    request_type: type[communication.Message] = Settings
    response_type: type[communication.Message] = Acknowledge

    @override
    def execute(self, request: Settings) -> Acknowledge:
        settings = request
        _ = settings
        # TODO add logic
        return ACK


class CalcTrajectory(communication.Command):
    """Calculate trajectory using current attitude."""

    cmd: int = 3
    request_type: type[communication.Message] = EmptyMessage
    response_type: type[communication.Message] = Trajectory

    def __init__(
        self,
        attitude_filter: attitude_estimation.AttitudeFilter,
        trajectory_calculator: trajectory.TrajectoryCalculator,
    ):
        self._attitude_filter = attitude_filter
        self._trajectory_calculator = trajectory_calculator

    @override
    def execute(self, request: EmptyMessage) -> Trajectory:
        _ = request
        az_el = self._attitude_filter.get_azimuth_elevation()

        if az_el is None:
            return Trajectory(ack=NACK)

        az, el = az_el
        polynom_trajectory = self._trajectory_calculator(az, el)
        response = Trajectory(
            AcknowledgeEnum.ACK,
            polynom_trajectory.start,
            polynom_trajectory.stop,
            polynom_trajectory.position_coeffs.tolist(),
        )
        return response


class ShutdownInterruptError(Exception):
    """Used to signal a shutdown."""


class Shutdown(communication.Command):
    cmd: int = 4
    request_type: type[communication.Message] = EmptyMessage
    response_type: type[communication.Message] = Acknowledge

    def __init__(self, *, enable_shutdown: bool, shutdown_delay: float):
        """Quit the application.

        Args:
            enable_shutdown: Application is allowed to shutdown the system.
            shutdown_delay: Duration in seconds to wait before shutdown is triggered.
        """
        self._enable_shutdown = enable_shutdown
        self._shutdown_delay = shutdown_delay

    @override
    def execute(self, request: EmptyMessage) -> Never:
        _ = request
        if self._enable_shutdown:
            time.sleep(self._shutdown_delay)
            raise ShutdownInterruptError()
        raise KeyboardInterrupt()


class SetAttitudeEstimationMode(communication.Command):
    """Set the attitude estimation mode."""

    cmd: int = 5
    request_type: type[communication.Message] = AttitudeEstimationMode
    response_type: type[communication.Message] = Acknowledge

    def __init__(
        self,
        image_acquisitioner: attitude_estimation.ImageAcquisitioner,
    ):
        self._image_acquisitioner = image_acquisitioner

    @override
    def execute(self, request: AttitudeEstimationMode) -> Acknowledge:
        self._image_acquisitioner.mode = request.mode
        return ACK


class GetStars(communication.Command):
    """Get list of recognized star positions."""

    cmd: int = 6
    request_type: type[communication.Message] = EmptyMessage
    response_type: type[communication.Message] = Stars

    def __init__(
        self,
        image_acquisitioner: attitude_estimation.ImageAcquisitioner,
    ):
        self._image_acquisitioner = image_acquisitioner

    @override
    def execute(self, request: EmptyMessage) -> Stars:
        _ = request
        positions = self._image_acquisitioner.positions
        fixed_size_positions = np.full((MAX_STAR_COUNT, 2), 255, np.uint8)
        if positions is None:
            count = 0
        else:
            count = min(len(positions), MAX_STAR_COUNT)
            common_slc = slice(count)
            fixed_size_positions[common_slc] = np.round(positions * 128)[common_slc]
        response = Stars(count, fixed_size_positions)
        return response


class App:
    def __init__(self, *, ser: Optional[serial.Serial] = None, enable_shutdown: bool = True):
        """Production application class.

        Args:
            ser: Serial instance to use.
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
        tc = trajectory.TrajectoryCalculator(s.trajectory.max_seconds, s.trajectory.max_dist, ms)
        af = attitude_estimation.AttitudeFilter()
        self._ia = attitude_estimation.ImageAcquisitioner(af)

        # All available commands
        self._commands = [
            GetStatus(af, tc, self._ia),
            SetSettings(),
            CalcTrajectory(af, tc),
            Shutdown(enable_shutdown=enable_shutdown, shutdown_delay=s.shutdown_delay),
            SetAttitudeEstimationMode(self._ia),
            GetStars(self._ia),
        ]

        # Run communication handler to process incoming commands.
        self._ser = serial.Serial(config.settings.serial_port) if ser is None else ser

    def __enter__(self) -> Self:
        self._ia.__enter__()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self._ia.__exit__(exc_type, exc_value, traceback)

    def __call__(self) -> int:
        """Run main loop of the command handler."""
        s = communication.PacketHandler(self._ser)
        com_handler = communication.CommandHandler(s, self._commands, NACK)
        logging.info("Running")
        try:
            com_handler.run_indefinitely()
        except KeyboardInterrupt:
            logging.info("KeyboardInterrupt")
        except ShutdownInterruptError:
            logging.info("ShutdownInterrupt")
            return 5
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
        with App(enable_shutdown=args.enable_shutdown) as app:
            return app()
    except Exception:
        logging.critical("Unhandled exception", exc_info=sys.exc_info())
        return -1


if __name__ == "__main__":
    raise SystemExit(main())
