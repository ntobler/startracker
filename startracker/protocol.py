"""Protocol definitions for the UART connection to the STM32 main board."""

import pathlib
from typing import Union

from . import communication, main


def generate_code(c_file: Union[str, pathlib.Path] = "out.c"):
    c_file = pathlib.Path(c_file)
    h_file = c_file.parent / (c_file.stem + ".h")

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
        h_file,
    )

    commands: list[type[communication.Command]] = [
        main.GetStatus,
        main.SetSettings,
        main.CalcTrajectory,
        main.Shutdown,
        main.SetAttitudeEstimationMode,
        main.GetStars,
    ]

    with c_file.open("w") as f:
        for cmd in commands:
            f.write(cmd.generate_c_code())


if __name__ == "__main__":
    generate_code()
