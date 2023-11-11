"""Main entrypoint for the startracker application."""

from . import trajectory
from . import camera


def main() -> int:
    print("Currently i do nothing at all.")
    # Load some objects
    trajectory.PolynomTrajectory
    camera.Camera
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
