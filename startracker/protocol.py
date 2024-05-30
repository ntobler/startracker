"""Protocol definitions for the UART connection to the STM32 main board."""

import pathlib

from . import communication, main


def generate_code():
    """Utility function to generate code for the STM project."""
    communication.gen_code_with_dependencies(
        [
            main.Acknowledge,
            main.Settings,
            main.Trajectory,
            main.Status,
            main.EmptyMessage,
            main.AttitudeEstimationMode,
            main.Stars,
        ],
        pathlib.Path("out.h"),
    )

    commands = [
        main.GetStatus,
        main.SetSettings,
        main.CalcTrajectory,
        main.Shutdown,
        main.SetAttitudeEstimationMode,
        main.GetStars,
    ]

    commands_c_file = pathlib.Path("commands.c")

    with commands_c_file.open("w") as f:
        for cmd in commands:
            f.write(cmd.generate_c_code())


if __name__ == "__main__":
    generate_code()
